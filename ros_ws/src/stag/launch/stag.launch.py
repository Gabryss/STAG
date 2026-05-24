from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _declare(name: str, default_value: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def _bool(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _int(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _float(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("stag"), "config", "stag.yaml"]
    )
    arguments = [
        _declare("params_file", default_params, "YAML file with node parameters."),
        _declare("rover_name", "rover", "Rover namespace used for default topic names."),
        _declare("costmap_topic", "", "Input nav_msgs/OccupancyGrid topic. Empty uses /<rover_name>/costmap."),
        _declare("graph_topic", "", "Output TerrainGraph topic. Empty uses /<rover_name>/stag_graph."),
        _declare("marker_topic", "", "Output MarkerArray topic. Empty uses /<rover_name>/stag_markers."),
        _declare("diagnostics_topic", "", "Output JSON diagnostics topic. Empty uses /<rover_name>/stag_diagnostics."),
        _declare("save_service", "", "Save Trigger service. Empty uses /<rover_name>/save_stag_graph."),
        _declare("recompute_service", "", "Recompute Trigger service. Empty uses /<rover_name>/recompute_stag_graph."),
        _declare("free_threshold", "49", "Occupancy values <= this are considered traversable."),
        _declare("unknown_is_obstacle", "true", "Treat unknown OccupancyGrid cells as blocked."),
        _declare("keep_largest_component", "true", "Drop disconnected smaller free-space components."),
        _declare("clearance_min_cells", "1.0", "Minimum edge clearance in costmap cells."),
        _declare("simplify_samples_per_cell", "4.0", "Samples per cell for simplified edge validation."),
        _declare("prune_leaf_length_cells", "8.0", "Prune dead-end edges shorter than this many cells."),
        _declare("prune_leaf_to_radius_ratio", "2.0", "Prune dead ends shorter than ratio * mean clearance."),
        _declare("gradient_node_count", "48", "Maximum number of gradient nodes to add."),
        _declare("gradient_node_quantile", "0.86", "Gradient magnitude quantile threshold."),
        _declare("gradient_min_separation_cells", "10.0", "Minimum spacing between gradient nodes in cells."),
        _declare("seed", "0", "Deterministic seed for medial-axis tie breaking."),
        _declare("min_publish_period_sec", "0.0", "Optional minimum period between graph publications."),
        _declare("publish_markers", "true", "Publish RViz/Foxglove MarkerArray output."),
        _declare("publish_diagnostics", "true", "Publish JSON diagnostics output."),
        _declare("save_graph", "true", "Save the latest graph snapshot to disk."),
        _declare("save_period_sec", "60.0", "Periodic save interval in seconds."),
        _declare("graph_save_path", "", "Output .npz path. Empty uses /tmp/<rover_name>_stag_latest.npz."),
        _declare("costmap_qos_reliability", "best_effort", "Costmap QoS reliability: best_effort or reliable."),
        _declare("graph_qos_reliability", "reliable", "Graph QoS reliability: best_effort or reliable."),
        _declare("graph_transient_local", "true", "Use transient local durability for graph/marker/diagnostic publishers."),
        _declare("graph_qos_depth", "1", "Graph, marker, and diagnostic publisher QoS depth."),
        _declare("costmap_qos_depth", "1", "Costmap subscriber QoS depth."),
        _declare("marker_node_scale", "0.18", "RViz node marker scale in meters."),
        _declare("marker_edge_width", "0.05", "RViz edge marker width in meters."),
        _declare("marker_color_metric", "traversability", "Edge marker colors: traversability, clearance, or uniform."),
    ]

    node = Node(
        package="stag",
        executable="stag_node",
        name="stag_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "rover_name": LaunchConfiguration("rover_name"),
                "costmap_topic": LaunchConfiguration("costmap_topic"),
                "graph_topic": LaunchConfiguration("graph_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                "save_service": LaunchConfiguration("save_service"),
                "recompute_service": LaunchConfiguration("recompute_service"),
                "free_threshold": _int("free_threshold"),
                "unknown_is_obstacle": _bool("unknown_is_obstacle"),
                "keep_largest_component": _bool("keep_largest_component"),
                "clearance_min_cells": _float("clearance_min_cells"),
                "simplify_samples_per_cell": _float("simplify_samples_per_cell"),
                "prune_leaf_length_cells": _float("prune_leaf_length_cells"),
                "prune_leaf_to_radius_ratio": _float("prune_leaf_to_radius_ratio"),
                "gradient_node_count": _int("gradient_node_count"),
                "gradient_node_quantile": _float("gradient_node_quantile"),
                "gradient_min_separation_cells": _float("gradient_min_separation_cells"),
                "seed": _int("seed"),
                "min_publish_period_sec": _float("min_publish_period_sec"),
                "publish_markers": _bool("publish_markers"),
                "publish_diagnostics": _bool("publish_diagnostics"),
                "save_graph": _bool("save_graph"),
                "save_period_sec": _float("save_period_sec"),
                "graph_save_path": LaunchConfiguration("graph_save_path"),
                "costmap_qos_reliability": LaunchConfiguration("costmap_qos_reliability"),
                "graph_qos_reliability": LaunchConfiguration("graph_qos_reliability"),
                "graph_transient_local": _bool("graph_transient_local"),
                "graph_qos_depth": _int("graph_qos_depth"),
                "costmap_qos_depth": _int("costmap_qos_depth"),
                "marker_node_scale": _float("marker_node_scale"),
                "marker_edge_width": _float("marker_edge_width"),
                "marker_color_metric": LaunchConfiguration("marker_color_metric"),
            },
        ],
    )

    return LaunchDescription([*arguments, node])
