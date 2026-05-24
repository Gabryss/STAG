# STAG: Sparse Traversability-Aware Topological Graphs

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD--3--Clause-yellow.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy%20tested-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)

STAG is a ROS 2 package for converting rover occupancy-grid costmaps into sparse, traversability-aware topological graphs. It is designed for navigation stacks that need a compact graph representation without throwing away terrain cost, clearance, and transition information.

The core node subscribes to a `nav_msgs/msg/OccupancyGrid`, extracts a multi-source terrain graph, publishes a custom `stag/msg/TerrainGraph`, emits RViz markers, publishes compact JSON diagnostics, and can persist compressed `.npz` graph snapshots for later analysis.

## What It Builds

STAG combines three complementary graph sources:

- **Topology graph**: medial-axis skeleton of free space, with endpoints, junctions, loop anchors, edge polylines, and clearance metrics.
- **Region graph**: representative medoid nodes for cost-homogeneous traversability regions.
- **Gradient graph**: nodes near strong cost transitions and high-gradient terrain features.

The resulting graph stays much smaller than a grid representation while preserving the information a planner usually needs for risk-aware route selection:

- node source/type metadata
- node degree
- node traversability
- edge length
- minimum and mean clearance
- minimum and mean edge traversability
- edge waypoint polylines
- source-aware connector edges between topology, region, and gradient nodes

## Repository Layout

```text
.
├── Dockerfile
├── docker-compose.yaml
├── docker/
│   ├── README.md
│   └── entrypoint.sh
├── ros_ws/src/stag/
│   ├── config/stag.yaml
│   ├── launch/stag.launch.py
│   ├── msg/
│   ├── scripts/stag_node
│   ├── stag_core/
│   └── test/
├── requirements-dev.txt
└── README.md
```

Detailed package documentation lives in [ros_ws/src/stag/README.md](ros_ws/src/stag/README.md). Docker-specific notes are in [docker/README.md](docker/README.md).

## Topics And Services

Default names are derived from `rover_name`, which defaults to `rover`.

| Interface | Default name | Type |
|---|---|---|
| Input costmap | `/<rover_name>/costmap` | `nav_msgs/msg/OccupancyGrid` |
| Output graph | `/<rover_name>/stag_graph` | `stag/msg/TerrainGraph` |
| RViz markers | `/<rover_name>/stag_markers` | `visualization_msgs/msg/MarkerArray` |
| Diagnostics | `/<rover_name>/stag_diagnostics` | `std_msgs/msg/String` JSON |
| Save graph | `/<rover_name>/save_stag_graph` | `std_srvs/srv/Trigger` |
| Recompute graph | `/<rover_name>/recompute_stag_graph` | `std_srvs/srv/Trigger` |

All topic and service names can be overridden through launch arguments or ROS parameters.

## Quick Start

From a ROS 2 workspace containing this package:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select stag
source install/setup.bash
ros2 launch stag stag.launch.py rover_name:=my_rover
```

This subscribes to:

```text
/my_rover/costmap
```

and publishes:

```text
/my_rover/stag_graph
/my_rover/stag_markers
/my_rover/stag_diagnostics
```

You can also run the node directly:

```bash
ros2 run stag stag_node --ros-args \
  -p rover_name:=my_rover \
  -p costmap_topic:=/robot_1/local_costmap/costmap \
  -p graph_topic:=/robot_1/stag_graph
```

For a self-contained smoke demo, run STAG in one terminal and publish a synthetic cross-shaped costmap in another:

```bash
ros2 launch stag stag.launch.py rover_name:=rover
ros2 run stag demo_costmap_publisher -- --rover-name rover
```

## Docker

Build and run the controlled ROS environment:

```bash
docker compose build
docker compose up stag
```

Compose launches:

```bash
ros2 launch stag stag.launch.py \
  params_file:=/config/stag.yaml \
  graph_save_path:=/graph_output/stag_latest.npz
```

The container mounts [ros_ws/src/stag/config/stag.yaml](ros_ws/src/stag/config/stag.yaml) as `/config/stag.yaml`. Saved graph snapshots are written to `docker/graph_output/` on the host.

Override the ROS domain when needed:

```bash
ROS_DOMAIN_ID=7 docker compose up stag
```

## Configuration

The default parameter file is [ros_ws/src/stag/config/stag.yaml](ros_ws/src/stag/config/stag.yaml). Common tuning parameters include:

- `free_threshold`: occupancy values at or below this value are traversable.
- `unknown_is_obstacle`: whether `-1` cells are blocked.
- `keep_largest_component`: filter disconnected free-space islands.
- `clearance_min_cells`: minimum clearance required for simplified edges.
- `prune_leaf_length_cells`: remove short dead-end branches.
- `gradient_node_count`: maximum number of gradient nodes to add.
- `gradient_min_separation_cells`: spacing between gradient nodes.
- `min_publish_period_sec`: throttle expensive costmap updates.
- `publish_markers`: disable marker construction/publishing on embedded systems.
- `publish_diagnostics`: disable diagnostics publishing when not needed.
- `marker_color_metric`: color graph edges by `traversability`, `clearance`, or `uniform`.
- `save_graph`, `save_period_sec`, `graph_save_path`: graph snapshot behavior.
- `costmap_qos_reliability`, `graph_qos_reliability`, `graph_transient_local`: ROS QoS behavior.

The node validates parameter ranges at startup and fails fast for invalid thresholds, negative counts, invalid quantiles, non-positive sampling density, invalid QoS reliability strings, or invalid marker sizes.
The same validation is applied to runtime parameter updates.

Example launch override:

```bash
ros2 launch stag stag.launch.py \
  rover_name:=my_rover \
  free_threshold:=49 \
  unknown_is_obstacle:=true \
  clearance_min_cells:=1.0 \
  gradient_node_count:=48 \
  graph_save_path:=/tmp/my_rover_stag_latest.npz
```

## Outputs

`stag/msg/TerrainGraph` contains:

- map metadata: header, width, height, resolution
- `GraphNode[] nodes`
- `GraphEdge[] edges`

Each node includes an ID, world-frame position, source, type, degree, and traversability score in `[0, 1]`. Each edge includes an ID, start/end node indices, length, minimum clearance, mean clearance, minimum traversability, mean traversability, and a world-frame polyline.

Diagnostics are published as compact JSON with map size, open-cell count, graph counts, source counts, extraction time, moving-average and maximum extraction time, processed graph count, received/skipped/dropped costmap counts, latest input stamp, and save path.

When snapshot saving is enabled, the latest graph is saved atomically as a compressed `.npz` file. The node writes a temporary file beside the target path and renames it into place.

Costmap callbacks are intentionally lightweight: the newest costmap is stored and graph extraction runs in a background worker. If updates arrive faster than STAG can process them, stale pending costmaps are dropped and counted in diagnostics.

## Benchmarking

Run the synthetic core benchmark to estimate extraction cost on your target hardware:

```bash
ros2 run stag benchmark_topology_core -- --sizes 100 250 500 --runs 5
```

For Raspberry Pi deployments, start with `min_publish_period_sec: 1.0`, `publish_markers: false`, and a lower `gradient_node_count`, then use diagnostics and benchmark results to tune upward.

## Testing

Install local Python test dependencies when running tests outside a ROS dependency-managed environment:

```bash
python3 -m pip install -r requirements-dev.txt
```

Run unit tests directly:

```bash
python3 -m pytest ros_ws/src/stag/test/test_topology_core.py -v
```

Run through colcon:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stag --base-paths ros_ws/src
colcon test --packages-select stag --base-paths ros_ws/src
```

The current test suite covers empty maps, fully blocked grids, corridor and junction extraction, unknown-cell handling, auxiliary node extraction, compact polyline serialization, and a ROS node integration path that publishes a costmap and verifies graph, marker, diagnostics, and recompute-service behavior.

## Development Notes

- ROS package name: `stag`
- Main executable: `stag_node`
- Demo executable: `demo_costmap_publisher`
- Benchmark executable: `benchmark_topology_core`
- Python package: `stag_core`
- Custom messages: `GraphNode`, `GraphEdge`, `TerrainGraph`
- Primary algorithm module: [ros_ws/src/stag/stag_core/topology_core.py](ros_ws/src/stag/stag_core/topology_core.py)
- ROS integration module: [ros_ws/src/stag/stag_core/stag_node.py](ros_ws/src/stag/stag_core/stag_node.py)
- CI: [.github/workflows/ros-ci.yaml](.github/workflows/ros-ci.yaml) builds/tests the ROS package and builds the Docker image on GitHub Actions.

Generated ROS build output (`build/`, `install/`, `log/`), Python caches, and graph snapshots are ignored by `.gitignore`.

## License

This project is licensed under the BSD 3-Clause License. See [LICENSE](LICENSE).
