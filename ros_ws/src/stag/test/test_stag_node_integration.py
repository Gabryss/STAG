from __future__ import annotations

import json
from pathlib import Path
import time
import unittest

import numpy as np
import rclpy
from rclpy.context import Context
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from stag.msg import TerrainGraph
from stag_core.stag_node import StagNode


def _demo_costmap() -> OccupancyGrid:
    width = 48
    height = 48
    values = np.full((height, width), 100, dtype=np.int8)
    values[22:26, 6:42] = 5
    values[6:42, 22:26] = 20
    values[17:31, 17:31] = 0

    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = 0.1
    message.info.width = width
    message.info.height = height
    message.info.origin.orientation.w = 1.0
    message.data = values.ravel().astype(int).tolist()
    return message


class StagNodeIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        self.context.init(
            args=[
                "--ros-args",
                "-p",
                "gradient_node_count:=4",
                "-p",
                "simplify_samples_per_cell:=2.0",
                "-p",
                "save_graph:=false",
            ],
            initialize_logging=False,
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.stag_node = StagNode(context=self.context)
        self.client_node = Node("stag_integration_test_client", context=self.context)
        self.executor.add_node(self.stag_node)
        self.executor.add_node(self.client_node)

    def tearDown(self):
        self.executor.remove_node(self.client_node)
        self.executor.remove_node(self.stag_node)
        self.client_node.destroy_node()
        self.stag_node.destroy_node()
        self.context.shutdown()

    def _spin_until(self, predicate, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_node_publishes_graph_markers_diagnostics_and_recomputes(self):
        received_graphs: list[TerrainGraph] = []
        received_markers: list[MarkerArray] = []
        received_diagnostics: list[String] = []

        self.client_node.create_subscription(TerrainGraph, "/rover/stag_graph", received_graphs.append, 1)
        self.client_node.create_subscription(MarkerArray, "/rover/stag_markers", received_markers.append, 1)
        self.client_node.create_subscription(String, "/rover/stag_diagnostics", received_diagnostics.append, 1)
        publisher = self.client_node.create_publisher(OccupancyGrid, "/rover/costmap", 1)

        costmap = _demo_costmap()
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not (received_graphs and received_markers and received_diagnostics):
            costmap.header.stamp = self.client_node.get_clock().now().to_msg()
            publisher.publish(costmap)
            self.executor.spin_once(timeout_sec=0.1)

        self.assertTrue(received_graphs, "STAG graph was not published")
        self.assertTrue(received_markers, "RViz markers were not published")
        self.assertTrue(received_diagnostics, "diagnostics were not published")
        self.assertTrue(received_markers[-1].markers[1].colors, "edge marker colors were not populated")

        graph = received_graphs[-1]
        self.assertEqual(graph.header.frame_id, "map")
        self.assertGreater(len(graph.nodes), 0)
        self.assertGreater(len(graph.edges), 0)
        self.assertTrue(all(0.0 <= node.traversability <= 1.0 for node in graph.nodes))
        self.assertTrue(all(0.0 <= edge.min_traversability <= edge.mean_traversability <= 1.0 for edge in graph.edges))

        diagnostics = json.loads(received_diagnostics[-1].data)
        self.assertEqual(diagnostics["graph_nodes"], len(graph.nodes))
        self.assertEqual(diagnostics["graph_edges"], len(graph.edges))
        self.assertGreaterEqual(diagnostics["graphs_processed"], 1)
        self.assertIn("extraction_time_avg_ms", diagnostics)
        self.assertIn("costmaps_received", diagnostics)

        invalid_results = self.stag_node.set_parameters(
            [Parameter("gradient_node_quantile", Parameter.Type.DOUBLE, 2.0)]
        )
        self.assertFalse(invalid_results[0].successful)
        valid_results = self.stag_node.set_parameters(
            [Parameter("marker_color_metric", Parameter.Type.STRING, "clearance")]
        )
        self.assertTrue(valid_results[0].successful)

        recompute_client = self.client_node.create_client(Trigger, "/rover/recompute_stag_graph")
        self.assertTrue(self._spin_until(recompute_client.service_is_ready), "recompute service was not available")
        future = recompute_client.call_async(Trigger.Request())
        self.assertTrue(self._spin_until(future.done), "recompute service did not respond")
        self.assertTrue(future.result().success)

        save_client = self.client_node.create_client(Trigger, "/rover/save_stag_graph")
        self.assertTrue(self._spin_until(save_client.service_is_ready), "save service was not available")
        future = save_client.call_async(Trigger.Request())
        self.assertTrue(self._spin_until(future.done), "save service did not respond")
        self.assertTrue(future.result().success)
        saved_path = Path("/tmp/rover_stag_latest.npz")
        self.assertTrue(saved_path.exists())
        saved_path.unlink()


class StagNodeParameterValidationTest(unittest.TestCase):
    def tearDown(self):
        if hasattr(self, "context") and self.context.ok():
            self.context.shutdown()

    def test_invalid_parameter_fails_fast(self):
        self.context = Context()
        self.context.init(args=["--ros-args", "-p", "free_threshold:=200"], initialize_logging=False)
        with self.assertRaisesRegex(ValueError, "free_threshold"):
            StagNode(context=self.context)


if __name__ == "__main__":
    unittest.main()
