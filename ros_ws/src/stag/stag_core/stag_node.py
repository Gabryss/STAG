from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from stag.msg import GraphEdge, GraphNode, TerrainGraph
from stag_core.topology_core import (
    NODE_SOURCE_GRADIENT,
    NODE_SOURCE_REGION,
    NODE_SOURCE_TOPOLOGY,
    NODE_TYPE_GRADIENT,
    NODE_TYPE_REGION_MEDOID,
    analyze_costmap_topology,
    extract_auxiliary_graph_nodes,
    flatten_polylines,
    occupancy_grid_to_open_mask,
    occupancy_grid_to_traversability,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    color = ColorRGBA()
    color.r = float(r)
    color.g = float(g)
    color.b = float(b)
    color.a = float(a)
    return color


class StagNode(Node):
    def __init__(self, *, context: Context | None = None) -> None:
        super().__init__("stag_node", context=context)
        self._declare_parameters()
        self._validate_parameters()
        self.add_on_set_parameters_callback(self._handle_parameter_update)

        rover_name = self._string_parameter("rover_name")
        costmap_topic = self._string_parameter("costmap_topic") or f"/{rover_name}/costmap"
        graph_topic = self._string_parameter("graph_topic") or f"/{rover_name}/stag_graph"
        marker_topic = self._string_parameter("marker_topic") or f"/{rover_name}/stag_markers"
        diagnostics_topic = self._string_parameter("diagnostics_topic") or f"/{rover_name}/stag_diagnostics"
        save_service = self._string_parameter("save_service") or f"/{rover_name}/save_stag_graph"
        recompute_service = self._string_parameter("recompute_service") or f"/{rover_name}/recompute_stag_graph"

        graph_save_path = self._string_parameter("graph_save_path")
        if not graph_save_path:
            graph_save_path = f"/tmp/{rover_name}_stag_latest.npz"
        self._graph_save_path = Path(graph_save_path)

        graph_qos = self._publisher_qos()
        costmap_qos = self._costmap_qos()
        self._graph_pub = self.create_publisher(TerrainGraph, graph_topic, graph_qos)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, graph_qos)
        self._diagnostics_pub = self.create_publisher(String, diagnostics_topic, graph_qos)
        self._costmap_sub = self.create_subscription(OccupancyGrid, costmap_topic, self._handle_costmap, costmap_qos)
        self._save_srv = self.create_service(Trigger, save_service, self._handle_save_service)
        self._recompute_srv = self.create_service(Trigger, recompute_service, self._handle_recompute_service)

        self._costmap_lock = threading.Lock()
        self._costmap_event = threading.Event()
        self._worker_stop = threading.Event()
        self._last_costmap: OccupancyGrid | None = None
        self._pending_costmap: OccupancyGrid | None = None
        self._force_recompute_pending = False
        self._last_publish_time = 0.0
        self._last_save_time = 0.0
        self._latest_snapshot: dict[str, np.ndarray] | None = None
        self._latest_diagnostics: dict[str, object] = {}
        self._costmaps_received = 0
        self._costmaps_skipped_throttle = 0
        self._costmaps_dropped_stale = 0
        self._graphs_processed = 0
        self._avg_extraction_time_ms = 0.0
        self._max_extraction_time_ms = 0.0
        self._last_input_stamp_sec = 0
        self._last_input_stamp_nanosec = 0
        self._save_timer = self.create_timer(1.0, self._save_periodic_snapshot)
        self._worker = threading.Thread(target=self._processing_loop, name="stag_graph_worker", daemon=True)
        self._worker.start()

        self.get_logger().info(f"Subscribing to costmap: {costmap_topic}")
        self.get_logger().info(f"Publishing stag graph: {graph_topic}")
        self.get_logger().info(f"Publishing RViz markers: {marker_topic}")
        self.get_logger().info(f"Publishing diagnostics: {diagnostics_topic}")
        self.get_logger().info(f"Save service: {save_service}")
        self.get_logger().info(f"Recompute service: {recompute_service}")
        self.get_logger().info(f"Saving latest graph snapshot to: {self._graph_save_path}")

    def _declare_parameters(self) -> None:
        self.declare_parameter("rover_name", "rover")
        self.declare_parameter("costmap_topic", "")
        self.declare_parameter("graph_topic", "")
        self.declare_parameter("marker_topic", "")
        self.declare_parameter("diagnostics_topic", "")
        self.declare_parameter("save_service", "")
        self.declare_parameter("recompute_service", "")
        self.declare_parameter("free_threshold", 49)
        self.declare_parameter("unknown_is_obstacle", True)
        self.declare_parameter("keep_largest_component", True)
        self.declare_parameter("clearance_min_cells", 1.0)
        self.declare_parameter("simplify_samples_per_cell", 4.0)
        self.declare_parameter("prune_leaf_length_cells", 8.0)
        self.declare_parameter("prune_leaf_to_radius_ratio", 2.0)
        self.declare_parameter("gradient_node_count", 48)
        self.declare_parameter("gradient_node_quantile", 0.86)
        self.declare_parameter("gradient_min_separation_cells", 10.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("min_publish_period_sec", 0.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_diagnostics", True)
        self.declare_parameter("save_graph", True)
        self.declare_parameter("save_period_sec", 60.0)
        self.declare_parameter("graph_save_path", "")
        self.declare_parameter("costmap_qos_reliability", "best_effort")
        self.declare_parameter("graph_qos_reliability", "reliable")
        self.declare_parameter("graph_transient_local", True)
        self.declare_parameter("graph_qos_depth", 1)
        self.declare_parameter("costmap_qos_depth", 1)
        self.declare_parameter("marker_node_scale", 0.18)
        self.declare_parameter("marker_edge_width", 0.05)
        self.declare_parameter("marker_color_metric", "traversability")

    def _string_parameter(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).get_parameter_value().integer_value)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    def _bool_parameter(self, name: str) -> bool:
        return bool(self.get_parameter(name).get_parameter_value().bool_value)

    def _parameter_values(self) -> dict[str, object]:
        return {
            "free_threshold": self._int_parameter("free_threshold"),
            "clearance_min_cells": self._float_parameter("clearance_min_cells"),
            "simplify_samples_per_cell": self._float_parameter("simplify_samples_per_cell"),
            "prune_leaf_length_cells": self._float_parameter("prune_leaf_length_cells"),
            "prune_leaf_to_radius_ratio": self._float_parameter("prune_leaf_to_radius_ratio"),
            "gradient_node_count": self._int_parameter("gradient_node_count"),
            "gradient_node_quantile": self._float_parameter("gradient_node_quantile"),
            "gradient_min_separation_cells": self._float_parameter("gradient_min_separation_cells"),
            "min_publish_period_sec": self._float_parameter("min_publish_period_sec"),
            "save_period_sec": self._float_parameter("save_period_sec"),
            "graph_qos_depth": self._int_parameter("graph_qos_depth"),
            "costmap_qos_depth": self._int_parameter("costmap_qos_depth"),
            "marker_node_scale": self._float_parameter("marker_node_scale"),
            "marker_edge_width": self._float_parameter("marker_edge_width"),
            "costmap_qos_reliability": self._string_parameter("costmap_qos_reliability"),
            "graph_qos_reliability": self._string_parameter("graph_qos_reliability"),
            "marker_color_metric": self._string_parameter("marker_color_metric"),
        }

    def _validate_parameter_values(self, values: dict[str, object]) -> list[str]:
        errors: list[str] = []

        def require(condition: bool, message: str) -> None:
            if not condition:
                errors.append(message)

        require(0 <= int(values["free_threshold"]) <= 100, "free_threshold must be in [0, 100]")
        require(float(values["clearance_min_cells"]) >= 0.0, "clearance_min_cells must be >= 0")
        require(float(values["simplify_samples_per_cell"]) > 0.0, "simplify_samples_per_cell must be > 0")
        require(float(values["prune_leaf_length_cells"]) >= 0.0, "prune_leaf_length_cells must be >= 0")
        require(float(values["prune_leaf_to_radius_ratio"]) >= 0.0, "prune_leaf_to_radius_ratio must be >= 0")
        require(int(values["gradient_node_count"]) >= 0, "gradient_node_count must be >= 0")
        require(0.0 <= float(values["gradient_node_quantile"]) <= 1.0, "gradient_node_quantile must be in [0, 1]")
        require(float(values["gradient_min_separation_cells"]) >= 0.0, "gradient_min_separation_cells must be >= 0")
        require(float(values["min_publish_period_sec"]) >= 0.0, "min_publish_period_sec must be >= 0")
        require(float(values["save_period_sec"]) >= 0.0, "save_period_sec must be >= 0")
        require(int(values["graph_qos_depth"]) >= 1, "graph_qos_depth must be >= 1")
        require(int(values["costmap_qos_depth"]) >= 1, "costmap_qos_depth must be >= 1")
        require(float(values["marker_node_scale"]) > 0.0, "marker_node_scale must be > 0")
        require(float(values["marker_edge_width"]) > 0.0, "marker_edge_width must be > 0")

        for parameter_name in ("costmap_qos_reliability", "graph_qos_reliability"):
            value = str(values[parameter_name]).strip().lower()
            require(
                value in ("reliable", "reliability_reliable", "best_effort", "besteffort", "best-effort", "reliability_best_effort"),
                f"{parameter_name} must be 'reliable' or 'best_effort'",
            )
        require(
            str(values["marker_color_metric"]).strip().lower() in ("traversability", "clearance", "uniform"),
            "marker_color_metric must be 'traversability', 'clearance', or 'uniform'",
        )

        return errors

    def _validate_parameters(self) -> None:
        errors = self._validate_parameter_values(self._parameter_values())
        if errors:
            raise ValueError("Invalid STAG parameters: " + "; ".join(errors))

    def _handle_parameter_update(self, parameters: list[Parameter]) -> SetParametersResult:
        values = self._parameter_values()
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
        errors = self._validate_parameter_values(values)
        result = SetParametersResult()
        result.successful = not errors
        result.reason = "; ".join(errors)
        return result

    def _reliability(self, value: str) -> ReliabilityPolicy:
        normalized = value.strip().lower()
        if normalized in ("reliable", "reliability_reliable"):
            return ReliabilityPolicy.RELIABLE
        if normalized in ("best_effort", "besteffort", "best-effort", "reliability_best_effort"):
            return ReliabilityPolicy.BEST_EFFORT
        self.get_logger().warn(f"Unknown QoS reliability '{value}', using reliable.")
        return ReliabilityPolicy.RELIABLE

    def _publisher_qos(self) -> QoSProfile:
        qos = QoSProfile(
            depth=max(1, self.get_parameter("graph_qos_depth").get_parameter_value().integer_value),
            reliability=self._reliability(self._string_parameter("graph_qos_reliability")),
        )
        if self.get_parameter("graph_transient_local").get_parameter_value().bool_value:
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return qos

    def _costmap_qos(self) -> QoSProfile:
        return QoSProfile(
            depth=max(1, self.get_parameter("costmap_qos_depth").get_parameter_value().integer_value),
            reliability=self._reliability(self._string_parameter("costmap_qos_reliability")),
        )

    def _handle_costmap(self, costmap: OccupancyGrid) -> None:
        with self._costmap_lock:
            self._costmaps_received += 1
            if self._pending_costmap is not None:
                self._costmaps_dropped_stale += 1
            self._last_costmap = costmap
            self._pending_costmap = costmap
            self._last_input_stamp_sec = int(costmap.header.stamp.sec)
            self._last_input_stamp_nanosec = int(costmap.header.stamp.nanosec)
        self._costmap_event.set()

    def _handle_save_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._latest_snapshot is None:
            response.success = False
            response.message = "No graph snapshot is available yet."
            return response
        path = self.save_latest_snapshot()
        response.success = True
        response.message = f"Saved graph snapshot to {path}"
        return response

    def _handle_recompute_service(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._costmap_lock:
            if self._last_costmap is None:
                response.success = False
                response.message = "No costmap has been received yet."
                return response
            self._pending_costmap = self._last_costmap
            self._force_recompute_pending = True
        self._costmap_event.set()
        response.success = True
        response.message = "Queued STAG graph recompute."
        return response

    def _processing_loop(self) -> None:
        while not self._worker_stop.is_set():
            self._costmap_event.wait(timeout=0.1)
            if self._worker_stop.is_set():
                break
            costmap, force = self._take_pending_costmap()
            if costmap is None:
                self._costmap_event.clear()
                continue

            now = time.monotonic()
            min_period = self._float_parameter("min_publish_period_sec")
            if not force and min_period > 0.0 and now - self._last_publish_time < min_period:
                with self._costmap_lock:
                    self._costmaps_skipped_throttle += 1
                continue
            self._compute_and_publish(costmap)

    def _take_pending_costmap(self) -> tuple[OccupancyGrid | None, bool]:
        with self._costmap_lock:
            costmap = self._pending_costmap
            force = self._force_recompute_pending
            self._pending_costmap = None
            self._force_recompute_pending = False
            return costmap, force

    def _compute_and_publish(self, costmap: OccupancyGrid) -> None:
        start_time = time.perf_counter()
        width = int(costmap.info.width)
        height = int(costmap.info.height)
        if width <= 0 or height <= 0:
            self.get_logger().warn("Received empty costmap; skipping graph extraction.")
            return
        if not costmap.header.frame_id:
            self.get_logger().warn("Costmap header.frame_id is empty; graph frame will also be empty.")

        raw_data = np.asarray(costmap.data, dtype=np.int16)
        expected_size = width * height
        if raw_data.size != expected_size:
            self.get_logger().error(
                f"Costmap data length {raw_data.size} does not match width*height {expected_size}; "
                "skipping graph extraction."
            )
            return
        free_threshold = self.get_parameter("free_threshold").get_parameter_value().integer_value
        unknown_is_obstacle = self.get_parameter("unknown_is_obstacle").get_parameter_value().bool_value
        open_mask = occupancy_grid_to_open_mask(
            raw_data,
            width=width,
            height=height,
            free_threshold=int(free_threshold),
            unknown_is_obstacle=bool(unknown_is_obstacle),
        )
        traversability = occupancy_grid_to_traversability(raw_data, width=width, height=height, open_mask=open_mask)

        result = analyze_costmap_topology(
            open_mask,
            clearance_min_cells=self.get_parameter("clearance_min_cells").get_parameter_value().double_value,
            simplify_samples_per_cell=self.get_parameter("simplify_samples_per_cell").get_parameter_value().double_value,
            prune_leaf_length_cells=self.get_parameter("prune_leaf_length_cells").get_parameter_value().double_value,
            prune_leaf_to_radius_ratio=self.get_parameter("prune_leaf_to_radius_ratio").get_parameter_value().double_value,
            keep_largest_component=self.get_parameter("keep_largest_component").get_parameter_value().bool_value,
            seed=self.get_parameter("seed").get_parameter_value().integer_value,
        )
        auxiliary = extract_auxiliary_graph_nodes(
            traversability,
            open_mask,
            gradient_node_count=self.get_parameter("gradient_node_count").get_parameter_value().integer_value,
            gradient_node_quantile=self.get_parameter("gradient_node_quantile").get_parameter_value().double_value,
            gradient_min_separation_cells=self.get_parameter("gradient_min_separation_cells").get_parameter_value().double_value,
        )

        graph_msg, snapshot = self._build_graph_message(costmap, traversability, result, auxiliary)
        markers = self._build_marker_array(costmap, graph_msg) if self._bool_parameter("publish_markers") else None
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        self._update_timing_stats(elapsed_ms)
        diagnostics = self._build_diagnostics(costmap, open_mask, graph_msg, auxiliary, elapsed_ms)

        self._latest_snapshot = snapshot
        self._latest_diagnostics = diagnostics
        self._graph_pub.publish(graph_msg)
        if markers is not None:
            self._marker_pub.publish(markers)
        if self._bool_parameter("publish_diagnostics"):
            diagnostics_msg = String()
            diagnostics_msg.data = json.dumps(diagnostics, sort_keys=True)
            self._diagnostics_pub.publish(diagnostics_msg)
        self._last_publish_time = time.monotonic()

        self.get_logger().info(
            f"Published unified STAG graph with {len(graph_msg.nodes)} nodes and {len(graph_msg.edges)} edges "
            f"in {elapsed_ms:.1f} ms.",
            throttle_duration_sec=2.0,
        )

    def _build_graph_message(self, costmap: OccupancyGrid, traversability: np.ndarray, result, auxiliary) -> tuple[TerrainGraph, dict[str, np.ndarray]]:
        graph = TerrainGraph()
        graph.header = costmap.header
        graph.width = costmap.info.width
        graph.height = costmap.info.height
        graph.resolution = costmap.info.resolution

        node_positions: list[np.ndarray] = [position.astype(float, copy=True) for position in result.node_positions]
        node_sources: list[int] = [NODE_SOURCE_TOPOLOGY] * len(result.node_positions)
        node_types: list[int] = [int(value) for value in result.node_types]

        region_start = len(node_positions)
        for position_xy, region_class in zip(auxiliary.region_positions, auxiliary.region_classes):
            node_positions.append(position_xy.astype(float, copy=True))
            node_sources.append(NODE_SOURCE_REGION)
            node_types.append(NODE_TYPE_REGION_MEDOID)

        for position_xy in auxiliary.gradient_positions:
            node_positions.append(position_xy.astype(float, copy=True))
            node_sources.append(NODE_SOURCE_GRADIENT)
            node_types.append(NODE_TYPE_GRADIENT)

        edge_node_indices: list[tuple[int, int]] = [(int(start), int(end)) for start, end in result.edge_node_indices]
        edge_polylines: list[np.ndarray] = [polyline.astype(float, copy=True) for polyline in result.edge_polylines]
        edge_lengths: list[float] = [float(value) for value in result.edge_lengths]
        edge_min_clearances: list[float] = [float(value) for value in result.edge_min_clearances]
        edge_mean_clearances: list[float] = [float(value) for value in result.edge_mean_clearances]
        edge_min_traversabilities: list[float] = []
        edge_mean_traversabilities: list[float] = []
        for polyline in edge_polylines:
            min_traversability, mean_traversability = self._polyline_traversability(polyline, traversability)
            edge_min_traversabilities.append(min_traversability)
            edge_mean_traversabilities.append(mean_traversability)

        topology_positions = result.node_positions
        for node_id in range(region_start, len(node_positions)):
            connector = self._nearest_clear_connector(node_positions[node_id], topology_positions, result.distance_field)
            if connector is None:
                continue
            topology_node_id, min_clearance, mean_clearance = connector
            polyline = np.vstack([node_positions[node_id], topology_positions[topology_node_id]])
            edge_node_indices.append((node_id, int(topology_node_id)))
            edge_polylines.append(polyline)
            edge_lengths.append(float(np.hypot(*(polyline[1] - polyline[0]))))
            edge_min_clearances.append(min_clearance)
            edge_mean_clearances.append(mean_clearance)
            min_traversability, mean_traversability = self._polyline_traversability(polyline, traversability)
            edge_min_traversabilities.append(min_traversability)
            edge_mean_traversabilities.append(mean_traversability)

        node_degrees = np.zeros(len(node_positions), dtype=int)
        for start_node, end_node in edge_node_indices:
            node_degrees[int(start_node)] += 1
            node_degrees[int(end_node)] += 1

        world_positions = [self._grid_xy_to_world_point(costmap, position_xy) for position_xy in node_positions]
        node_traversabilities = [self._point_traversability(position_xy, traversability) for position_xy in node_positions]
        for node_id, world_position in enumerate(world_positions):
            node = GraphNode()
            node.id = int(node_id)
            node.position = world_position
            node.source = int(node_sources[node_id])
            node.type = int(node_types[node_id])
            node.degree = int(node_degrees[node_id])
            node.traversability = float(node_traversabilities[node_id])
            graph.nodes.append(node)

        for edge_id, (edge_nodes, polyline) in enumerate(zip(edge_node_indices, edge_polylines)):
            edge = GraphEdge()
            edge.id = int(edge_id)
            edge.start_node = int(edge_nodes[0])
            edge.end_node = int(edge_nodes[1])
            edge.length = float(edge_lengths[edge_id] * costmap.info.resolution)
            edge.min_clearance = float(edge_min_clearances[edge_id] * costmap.info.resolution)
            edge.mean_clearance = float(edge_mean_clearances[edge_id] * costmap.info.resolution)
            edge.min_traversability = float(edge_min_traversabilities[edge_id])
            edge.mean_traversability = float(edge_mean_traversabilities[edge_id])
            edge.polyline = [self._grid_xy_to_world_point(costmap, point_xy) for point_xy in polyline]
            graph.edges.append(edge)

        snapshot = self._build_snapshot(
            costmap,
            node_positions,
            world_positions,
            node_sources,
            node_types,
            node_degrees,
            node_traversabilities,
            edge_node_indices,
            edge_polylines,
            edge_lengths,
            edge_min_clearances,
            edge_mean_clearances,
            edge_min_traversabilities,
            edge_mean_traversabilities,
        )
        return graph, snapshot

    def _build_snapshot(
        self,
        costmap: OccupancyGrid,
        node_positions: list[np.ndarray],
        world_positions: list[Point],
        node_sources: list[int],
        node_types: list[int],
        node_degrees: np.ndarray,
        node_traversabilities: list[float],
        edge_node_indices: list[tuple[int, int]],
        edge_polylines: list[np.ndarray],
        edge_lengths: list[float],
        edge_min_clearances: list[float],
        edge_mean_clearances: list[float],
        edge_min_traversabilities: list[float],
        edge_mean_traversabilities: list[float],
    ) -> dict[str, np.ndarray]:
        polyline_points, polyline_offsets = flatten_polylines(edge_polylines)
        origin = costmap.info.origin
        return {
            "stamp_sec": np.asarray(costmap.header.stamp.sec, dtype=np.int64),
            "stamp_nanosec": np.asarray(costmap.header.stamp.nanosec, dtype=np.int64),
            "resolution": np.asarray(costmap.info.resolution, dtype=np.float32),
            "width": np.asarray(costmap.info.width, dtype=np.uint32),
            "height": np.asarray(costmap.info.height, dtype=np.uint32),
            "origin_position": np.asarray([origin.position.x, origin.position.y, origin.position.z], dtype=np.float32),
            "origin_orientation": np.asarray(
                [origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w],
                dtype=np.float32,
            ),
            "node_positions_grid": np.asarray(node_positions, dtype=np.float32),
            "node_positions_world": np.asarray([[p.x, p.y, p.z] for p in world_positions], dtype=np.float32),
            "node_sources": np.asarray(node_sources, dtype=np.uint8),
            "node_types": np.asarray(node_types, dtype=np.uint8),
            "node_degrees": node_degrees.astype(np.uint32, copy=False),
            "node_traversabilities": np.asarray(node_traversabilities, dtype=np.float32),
            "edge_node_indices": np.asarray(edge_node_indices, dtype=np.uint32),
            "edge_lengths_m": np.asarray(edge_lengths, dtype=np.float32) * float(costmap.info.resolution),
            "edge_min_clearances_m": np.asarray(edge_min_clearances, dtype=np.float32) * float(costmap.info.resolution),
            "edge_mean_clearances_m": np.asarray(edge_mean_clearances, dtype=np.float32) * float(costmap.info.resolution),
            "edge_min_traversabilities": np.asarray(edge_min_traversabilities, dtype=np.float32),
            "edge_mean_traversabilities": np.asarray(edge_mean_traversabilities, dtype=np.float32),
            "edge_polyline_points_grid": polyline_points.astype(np.float32, copy=False),
            "edge_polyline_offsets": polyline_offsets.astype(np.uint32, copy=False),
        }

    def _build_marker_array(self, costmap: OccupancyGrid, graph: TerrainGraph) -> MarkerArray:
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = costmap.header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        edge_marker = Marker()
        edge_marker.header = costmap.header
        edge_marker.ns = "topology_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = self.get_parameter("marker_edge_width").get_parameter_value().double_value
        edge_marker.color = _color(1.0, 1.0, 1.0, 1.0)
        for edge in graph.edges:
            color = self._edge_marker_color(edge)
            for start, end in zip(edge.polyline[:-1], edge.polyline[1:]):
                edge_marker.points.append(start)
                edge_marker.points.append(end)
                edge_marker.colors.append(color)
                edge_marker.colors.append(color)
        marker_array.markers.append(edge_marker)

        source_specs = (
            (NODE_SOURCE_TOPOLOGY, "topology_nodes", 2, _color(0.35, 1.0, 0.65, 1.0)),
            (NODE_SOURCE_REGION, "region_nodes", 3, _color(1.0, 1.0, 1.0, 1.0)),
            (NODE_SOURCE_GRADIENT, "gradient_nodes", 4, _color(0.45, 0.90, 1.0, 1.0)),
        )
        node_scale = self.get_parameter("marker_node_scale").get_parameter_value().double_value
        for source, namespace, marker_id, color in source_specs:
            marker = Marker()
            marker.header = costmap.header
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.scale.x = node_scale
            marker.scale.y = node_scale
            marker.scale.z = node_scale
            marker.color = color
            marker.points = [node.position for node in graph.nodes if node.source == source]
            marker_array.markers.append(marker)
        return marker_array

    def _build_diagnostics(
        self,
        costmap: OccupancyGrid,
        open_mask: np.ndarray,
        graph: TerrainGraph,
        auxiliary,
        elapsed_ms: float,
    ) -> dict[str, object]:
        source_counts = {
            "topology": sum(1 for node in graph.nodes if node.source == NODE_SOURCE_TOPOLOGY),
            "region": sum(1 for node in graph.nodes if node.source == NODE_SOURCE_REGION),
            "gradient": sum(1 for node in graph.nodes if node.source == NODE_SOURCE_GRADIENT),
        }
        return {
            "frame_id": costmap.header.frame_id,
            "width": int(costmap.info.width),
            "height": int(costmap.info.height),
            "resolution": float(costmap.info.resolution),
            "open_cells": int(np.count_nonzero(open_mask)),
            "graph_nodes": len(graph.nodes),
            "graph_edges": len(graph.edges),
            "node_sources": source_counts,
            "region_candidates": int(len(auxiliary.region_positions)),
            "gradient_candidates": int(len(auxiliary.gradient_positions)),
            "extraction_time_ms": float(elapsed_ms),
            "extraction_time_avg_ms": float(self._avg_extraction_time_ms),
            "extraction_time_max_ms": float(self._max_extraction_time_ms),
            "graphs_processed": int(self._graphs_processed),
            "costmaps_received": int(self._costmaps_received),
            "costmaps_skipped_throttle": int(self._costmaps_skipped_throttle),
            "costmaps_dropped_stale": int(self._costmaps_dropped_stale),
            "last_input_stamp_sec": int(self._last_input_stamp_sec),
            "last_input_stamp_nanosec": int(self._last_input_stamp_nanosec),
            "publish_markers": bool(self._bool_parameter("publish_markers")),
            "publish_diagnostics": bool(self._bool_parameter("publish_diagnostics")),
            "save_path": str(self._graph_save_path),
        }

    def _update_timing_stats(self, elapsed_ms: float) -> None:
        self._graphs_processed += 1
        if self._graphs_processed == 1:
            self._avg_extraction_time_ms = float(elapsed_ms)
            self._max_extraction_time_ms = float(elapsed_ms)
            return
        alpha = 0.1
        self._avg_extraction_time_ms = (1.0 - alpha) * self._avg_extraction_time_ms + alpha * float(elapsed_ms)
        self._max_extraction_time_ms = max(self._max_extraction_time_ms, float(elapsed_ms))
        min_period_ms = self._float_parameter("min_publish_period_sec") * 1000.0
        if min_period_ms > 0.0 and elapsed_ms > min_period_ms:
            self.get_logger().warn(
                f"STAG extraction time {elapsed_ms:.1f} ms exceeds min_publish_period_sec "
                f"budget {min_period_ms:.1f} ms.",
                throttle_duration_sec=5.0,
            )

    def _edge_marker_color(self, edge: GraphEdge) -> ColorRGBA:
        metric = self._string_parameter("marker_color_metric").strip().lower()
        if metric == "uniform":
            return _color(0.08, 0.10, 0.14, 0.85)
        if metric == "clearance":
            value = min(1.0, max(0.0, float(edge.min_clearance) / max(1e-6, float(edge.mean_clearance) * 2.0)))
        else:
            value = min(1.0, max(0.0, float(edge.min_traversability)))
        return _color(1.0 - value, value, 0.12, 0.9)

    def _nearest_clear_connector(
        self,
        position_xy: np.ndarray,
        topology_positions: np.ndarray,
        distance_field: np.ndarray,
    ) -> tuple[int, float, float] | None:
        if len(topology_positions) == 0:
            return None
        distances = np.sum((topology_positions - position_xy[None, :]) ** 2, axis=1)
        for node_index in np.argsort(distances):
            topology_xy = topology_positions[int(node_index)]
            sample_clearances = self._connector_clearances(position_xy, topology_xy, distance_field)
            if sample_clearances.size == 0:
                continue
            return int(node_index), float(sample_clearances.min()), float(sample_clearances.mean())
        return None

    def _connector_clearances(
        self,
        start_xy: np.ndarray,
        end_xy: np.ndarray,
        distance_field: np.ndarray,
    ) -> np.ndarray:
        samples_per_cell = self.get_parameter("simplify_samples_per_cell").get_parameter_value().double_value
        clearance_min_cells = self.get_parameter("clearance_min_cells").get_parameter_value().double_value
        steps = max(2, int(np.ceil(np.hypot(*(end_xy - start_xy)) * samples_per_cell)) + 1)
        ix = np.rint(np.linspace(float(start_xy[0]), float(end_xy[0]), steps)).astype(int)
        iy = np.rint(np.linspace(float(start_xy[1]), float(end_xy[1]), steps)).astype(int)
        height, width = distance_field.shape
        if np.any(ix < 0) or np.any(ix >= width) or np.any(iy < 0) or np.any(iy >= height):
            return np.empty((0,), dtype=float)
        clearances = distance_field[iy, ix]
        if not np.all(clearances >= clearance_min_cells):
            return np.empty((0,), dtype=float)
        return clearances.astype(float, copy=False)

    def _point_traversability(self, position_xy: np.ndarray, traversability: np.ndarray) -> float:
        x = int(np.rint(float(position_xy[0])))
        y = int(np.rint(float(position_xy[1])))
        height, width = traversability.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            return 0.0
        return float(traversability[y, x])

    def _polyline_traversability(self, polyline_xy: np.ndarray, traversability: np.ndarray) -> tuple[float, float]:
        if len(polyline_xy) == 0:
            return 0.0, 0.0

        samples: list[np.ndarray] = []
        for start_xy, end_xy in zip(polyline_xy[:-1], polyline_xy[1:]):
            steps = max(2, int(np.ceil(np.hypot(*(end_xy - start_xy)))) + 1)
            xs = np.rint(np.linspace(float(start_xy[0]), float(end_xy[0]), steps)).astype(int)
            ys = np.rint(np.linspace(float(start_xy[1]), float(end_xy[1]), steps)).astype(int)
            samples.append(np.column_stack([xs, ys]))
        if not samples:
            xy = np.rint(polyline_xy).astype(int)
        else:
            xy = np.vstack(samples)

        height, width = traversability.shape
        valid = (xy[:, 0] >= 0) & (xy[:, 0] < width) & (xy[:, 1] >= 0) & (xy[:, 1] < height)
        if not np.any(valid):
            return 0.0, 0.0
        values = traversability[xy[valid, 1], xy[valid, 0]]
        return float(values.min()), float(values.mean())

    def _save_periodic_snapshot(self) -> None:
        if not self.get_parameter("save_graph").get_parameter_value().bool_value:
            return
        if self._latest_snapshot is None:
            return
        now = time.monotonic()
        period = self.get_parameter("save_period_sec").get_parameter_value().double_value
        if period <= 0.0 or now - self._last_save_time >= period:
            self.save_latest_snapshot()

    def save_latest_snapshot(self) -> Path | None:
        if self._latest_snapshot is None:
            return None
        self._graph_save_path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            prefix=f".{self._graph_save_path.name}.",
            suffix=".npz",
            dir=self._graph_save_path.parent,
            delete=False,
        ) as handle:
            temp_path = Path(handle.name)
        try:
            np.savez_compressed(temp_path, **self._latest_snapshot)
            temp_path.replace(self._graph_save_path)
        finally:
            if temp_path.exists():
                temp_path.unlink()
        self._last_save_time = time.monotonic()
        return self._graph_save_path

    def _grid_xy_to_world_point(self, costmap: OccupancyGrid, xy: np.ndarray) -> Point:
        resolution = float(costmap.info.resolution)
        origin = costmap.info.origin
        yaw = _yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = (float(xy[0]) + 0.5) * resolution
        local_y = (float(xy[1]) + 0.5) * resolution
        point = Point()
        point.x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
        point.y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
        point.z = origin.position.z
        return point

    def destroy_node(self) -> bool:
        self._worker_stop.set()
        self._costmap_event.set()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = StagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_latest_snapshot()
        node.destroy_node()
        rclpy.shutdown()
