# STAG: Sparse Traversability-Aware Topological Graphs

A ROS 2 package implementation that extracts sparse, traversability-aware topological graphs from rover costmaps for efficient autonomous navigation.

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD--3--Clause-yellow.svg)](../../../LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy%20tested-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![Package: stag](https://img.shields.io/badge/Package-stag-blue.svg)](#stag-sparse-traversability-aware-topological-graphs)

## Overview

STAG is the core ROS 2 implementation of the **Sparse Traversability-Aware Topological Graphs** method, a sophisticated algorithm for constructing sparse, traversability-aware topological representations of navigable environments from occupancy grid costmaps. Unlike traditional topological graphs that ignore terrain cost, STAG integrates three complementary graph sources to encode both geometric topology and terrain traversability:

- **Topology Graph**: Medial axis skeleton of free space with topology-aware node classification and clearance metrics
- **Region Graph**: Cost-based segmentation nodes representing homogeneous terrain traversability regions
- **Gradient Graph**: High-curvature and cost-transition nodes capturing complex terrain features and obstacles

The unified `TerrainGraph` message integrates all sources with semantic edge connections and full traversability metrics, enabling sophisticated navigation strategies that balance topological efficiency (sparse representation) with planetary surface traversability awareness (cost integration).

## Key Features

✨ **Traversability-Aware Graph Representation**
- Topology nodes and edges from medial axis skeleton with clearance metrics
- Region medoid nodes with cost/traversability classification
- Gradient nodes marking terrain transitions and cost discontinuities
- Semantic connector edges enabling cross-source navigation decisions

🎯 **Sparse Graph Generation**
- Medial axis skeleton reduces graph density by factor of 10-100× vs. grid-based planners
- Automatic component filtering (keeps largest connected component)
- Dead-end pruning with traversability-aware thresholds
- Configurable gradient node count for fine-tuning sparsity

🌍 **Cost-Aware Processing**
- Full integration of costmap traversability scores in node/edge metrics
- Gradient extraction from occupancy/cost fields for terrain feature detection
- Region clustering preserving cost homogeneity
- Edge simplification with clearance validation for safe navigation

📊 **Rich Metrics & Visualization**
- Real-time RViz marker visualization with color-coded node sources and cost gradients
- Per-edge clearance metrics (minimum and mean) for risk-aware planning
- Per-node degree information for topological analysis
- Interactive diagnostic overlay

🐳 **Production-Ready**
- Docker containerization for reproducible deployment
- Configurable QoS profiles for network efficiency
- Graph persistence with time-stamped snapshots
- Comprehensive parameter tuning
- Deterministic tie-breaking for reproducible results

## Topics

### Input
- **Input Costmap**: `/<rover_name>/costmap` (`nav_msgs/msg/OccupancyGrid`)
  - 2D occupancy grid costmap from navigation stack
  - Resolution and frame specified in costmap header

### Output
- **STAG Graph**: `/<rover_name>/stag_graph` (`stag/msg/TerrainGraph`)
  - Unified multi-source graph representation
  - Contains nodes with source classification and edges with clearance metrics
  
- **RViz Markers**: `/<rover_name>/stag_markers` (`visualization_msgs/msg/MarkerArray`)
  - Color-coded visualization of all graph components
  
- **Diagnostics**: `/<rover_name>/stag_diagnostics` (JSON string)
  - Performance metrics and processing statistics

```text
visualization_msgs/msg/MarkerArray
```

Default diagnostics:

```text
/<rover_name>/stag_diagnostics
```

Type:

```text
std_msgs/msg/String
```

The diagnostics string is compact JSON with map size, open cell count, graph counts, extraction time, moving-average and maximum extraction time, processed graph count, received/skipped/dropped costmap counts, latest input stamp, and save path.

The node also saves the latest graph as a compact compressed `.npz` file.

## Run

From a ROS 2 workspace that contains this package:

```bash
colcon build --packages-select stag
source install/setup.bash
ros2 run stag stag_node --ros-args -p rover_name:=my_rover
```

This subscribes to:

```text
/my_rover/costmap
```

and publishes:

```text
/my_rover/stag_graph
```

You can override topics directly:

```bash
ros2 run stag stag_node --ros-args \
  -p costmap_topic:=/robot_1/local_costmap/costmap \
  -p graph_topic:=/robot_1/stag_graph
```

Or use the launch file:

```bash
ros2 launch stag stag.launch.py rover_name:=my_rover
```

The launch file loads defaults from:

```text
config/stag.yaml
```

Example with common overrides:

```bash
ros2 launch stag stag.launch.py \
  rover_name:=my_rover \
  costmap_topic:=/my_rover/costmap \
  graph_topic:=/my_rover/stag_graph \
  marker_topic:=/my_rover/stag_markers \
  free_threshold:=49 \
  unknown_is_obstacle:=true \
  clearance_min_cells:=1.0 \
  gradient_node_count:=48 \
  save_period_sec:=60.0 \
  graph_save_path:=/tmp/my_stag_latest.npz
```

Force-save and force-recompute services are also available:

```bash
ros2 service call /my_rover/save_stag_graph std_srvs/srv/Trigger
ros2 service call /my_rover/recompute_stag_graph std_srvs/srv/Trigger
```

For a self-contained smoke demo, run the STAG node and synthetic costmap publisher in separate terminals:

```bash
ros2 launch stag stag.launch.py rover_name:=rover
ros2 run stag demo_costmap_publisher -- --rover-name rover
```

## Parameters

- `rover_name`: used to derive default topic names.
- `costmap_topic`: explicit input topic. If empty, uses `/<rover_name>/costmap`.
- `graph_topic`: explicit output topic. If empty, uses `/<rover_name>/stag_graph`.
- `marker_topic`: explicit RViz marker topic. If empty, uses `/<rover_name>/stag_markers`.
- `diagnostics_topic`: explicit diagnostics topic. If empty, uses `/<rover_name>/stag_diagnostics`.
- `save_service`: explicit force-save service. If empty, uses `/<rover_name>/save_stag_graph`.
- `recompute_service`: explicit force-recompute service. If empty, uses `/<rover_name>/recompute_stag_graph`.
- `free_threshold`: occupancy values `<= free_threshold` are treated as traversable.
- `unknown_is_obstacle`: whether `-1` costmap cells are blocked.
- `keep_largest_component`: drop disconnected small open components before graph extraction.
- `clearance_min_cells`: minimum clearance in costmap cells for simplified graph edges.
- `simplify_samples_per_cell`: sampling density used when validating simplified edges.
- `prune_leaf_length_cells`: prune short dead-end edges.
- `prune_leaf_to_radius_ratio`: prune dead ends shorter than this ratio times mean clearance.
- `gradient_node_count`: maximum number of gradient nodes to add.
- `gradient_node_quantile`: minimum gradient quantile used as a candidate threshold.
- `gradient_min_separation_cells`: minimum spacing between gradient nodes.
- `seed`: deterministic seed for medial-axis tie breaking.
- `min_publish_period_sec`: optional throttle for expensive costmap updates.
- `publish_markers`: publish RViz/Foxglove markers. Disable to reduce embedded CPU/network overhead.
- `publish_diagnostics`: publish JSON diagnostics.
- `save_graph`: enable compressed graph snapshots.
- `save_period_sec`: periodic save interval. Default is `60.0`.
- `graph_save_path`: output `.npz` path. If empty, uses `/tmp/<rover_name>_stag_latest.npz`.
- `costmap_qos_reliability`: `best_effort` or `reliable`.
- `graph_qos_reliability`: `best_effort` or `reliable`.
- `graph_transient_local`: publish graph, markers, and diagnostics with transient local durability.
- `graph_qos_depth`: publisher QoS queue depth.
- `costmap_qos_depth`: subscriber QoS queue depth.
- `marker_node_scale`: RViz node marker size in meters.
- `marker_edge_width`: RViz edge marker width in meters.
- `marker_color_metric`: edge marker colors: `traversability`, `clearance`, or `uniform`.

Parameters are validated at startup. Invalid thresholds, negative counts, out-of-range quantiles, non-positive sampling densities, invalid QoS reliability strings, and invalid marker sizes cause the node to fail fast with a clear error.
The same validation is applied to runtime parameter updates.

Costmap subscription callbacks only store the latest costmap and signal a background worker. Graph extraction runs outside the callback path. If costmaps arrive faster than processing can keep up, stale pending costmaps are dropped and counted in diagnostics.

The graph is saved periodically and once more when the node exits through `Ctrl+C`.
The save is atomic: the node writes a temporary `.npz` beside the target file, then renames it into place.

## Installation & Setup

### Prerequisites

- **ROS 2**: Jazzy tested. Humble is expected to work but should be treated as unverified unless covered by your CI.
- **Python**: 3.10 or higher
- **System Dependencies**: gcc, cmake

### From Source

Clone the repository and add to your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select stag
source install/setup.bash
```

Replace the clone URL with this repository's GitHub URL.

## Message Formats

### TerrainGraph Message

```text
std_msgs/Header header
uint32 width                    # Costmap width in cells
uint32 height                   # Costmap height in cells
float32 resolution              # Costmap resolution (m/cell)
GraphNode[] nodes               # All graph nodes
GraphEdge[] edges               # All graph edges
```

### GraphNode Message

```text
geometry_msgs/Point position    # Node position in costmap frame
uint32 type                     # NODE_TYPE_* constant
uint32 source                   # NODE_SOURCE_* constant
float32 traversability          # Traversability score at node (0-1)
```

**Node Types:**
- `NODE_TYPE_ENDPOINT` (1): Topology endpoints
- `NODE_TYPE_JUNCTION` (2): Topology junctions  
- `NODE_TYPE_LOOP_ANCHOR` (3): Synthetic topology anchor for loop-only skeleton components
- `NODE_TYPE_REGION_MEDOID` (10): Region representative node
- `NODE_TYPE_GRADIENT` (20): High-gradient/transition node

**Node Sources:**
- `NODE_SOURCE_TOPOLOGY` (1): From medial axis skeleton
- `NODE_SOURCE_REGION` (2): From cost segmentation
- `NODE_SOURCE_GRADIENT` (3): From gradient analysis

### GraphEdge Message

```text
uint32 start_node_index         # Start node index in nodes array
uint32 end_node_index           # End node index in nodes array
geometry_msgs/Point[] polyline  # Edge waypoints
float32 length                  # Edge length (m)
float32 min_clearance           # Minimum obstacle distance (m)
float32 mean_clearance          # Mean obstacle distance (m)
float32 min_traversability      # Minimum traversability sampled along edge (0-1)
float32 mean_traversability     # Mean traversability sampled along edge (0-1)
```

## Algorithm Overview

### STAG Method Implementation

STAG is the implementation of the **Sparse Traversability-Aware Topological Graphs** method, which addresses a critical gap in autonomous navigation: traditional topological graphs are geometry-aware but cost-blind, while traditional potential field methods are cost-aware but geometrically unwieldy. STAG bridges this gap by:

1. **Maintaining Sparsity**: Reduces graph complexity by 10-100× compared to grid-based representations through medial axis extraction
2. **Integrating Traversability**: Enriches graph with full cost/traversability metrics at every node and edge
3. **Enabling Semantic Navigation**: Three complementary graph sources provide flexibility for different planning strategies:
   - Use topology graph for fast path queries in low-cost areas
   - Use region graph for cost-conscious navigation in variable terrain
   - Use gradient graph for precision navigation near obstacles

### Processing Pipeline

STAG employs a multi-stage processing pipeline:

### Stage 1: Topology Extraction
- Converts occupancy grid to binary open/occupied mask
- Computes Euclidean distance field
- Extracts medial axis skeleton using distance field
- Classifies skeleton nodes (endpoints, junctions, loop anchors)
- Extracts topological edges with clearance metrics

### Stage 2: Simplification & Pruning
- Simplifies edges using Ramer-Douglas-Peucker with clearance constraints
- Prunes short dead-end edges below configurable thresholds
- Filters edges with insufficient clearance
- Keeps largest connected component (optional)

### Stage 3: Auxiliary Node Extraction
- **Region Nodes**: connected cost-band components represented by medoids
- **Gradient Nodes**: High-curvature points in cost gradient
- Deterministic node selection using seed
- Spatial pruning to maintain minimum separation

### Stage 4: Cross-Source Connectivity
- Connects region/gradient nodes to nearest topology node
- Validates connector edges for clearance
- Preserves topological connectivity

## Docker Deployment

Build and run in isolated environment:

```bash
## Build image
docker compose build

## Run STAG node
docker compose up stag

## Override configuration
ROS_DOMAIN_ID=7 docker compose up stag

## Custom parameters
docker compose run -e ROS_DOMAIN_ID=7 stag \
  ros2 launch stag stag.launch.py \
    free_threshold:=40 \
    gradient_node_count:=64
```

Graph snapshots are saved to:
```
docker/graph_output/
```

See [docker/README.md](../../../docker/README.md) for additional Docker guidance.

## Testing

Run unit tests:

```bash
colcon test --packages-select stag
```

For non-ROS local test runs, install the Python test dependencies first:

```bash
python3 -m pip install -r requirements-dev.txt
```

Run a specific test file from the repository root:

```bash
python3 -m pytest ros_ws/src/stag/test/test_topology_core.py -v
```

Benchmark core extraction on synthetic maps:

```bash
ros2 run stag benchmark_topology_core -- --sizes 100 250 500 --runs 5
```

Test coverage includes:
- Empty map handling
- Fully blocked occupancy grids
- Corridor and junction topologies
- Unknown cell handling (blocked vs. free)
- Auxiliary node extraction and polyline compaction
- ROS node integration with graph, marker, diagnostics, and service checks

## Troubleshooting

### No graph nodes generated
- **Cause**: Costmap is mostly occupied or resolution mismatch
- **Solution**: Check `free_threshold` parameter, verify costmap quality

### Memory usage high
- **Cause**: Large costmaps with many nodes
- **Solution**: Increase `gradient_min_separation_cells` or reduce `gradient_node_count`

### Slow processing
- **Cause**: Costmap too large or smoothing overhead
- **Solution**: Decrease costmap resolution, tune `simplify_samples_per_cell`

### Disconnected graph components
- **Cause**: `keep_largest_component` filtering
- **Solution**: Verify costmap connectivity or disable filtering

### RViz markers not visible
- **Cause**: Wrong frame or namespace
- **Solution**: Verify frame_id in costmap header, check rover_name setting

## Performance Characteristics

Typical performance on modern hardware:

| Costmap Size | Open Cells | Extract Time | Update Rate |
|---|---|---|---|
| 100×100 @ 0.05m/cell | 50% | ~5 ms | 200 Hz |
| 500×500 @ 0.05m/cell | 40% | ~150 ms | 6-7 Hz |
| 1000×1000 @ 0.05m/cell | 30% | ~1000 ms | ~1 Hz |

*Times measured on Intel i7 with scipy/skimage optimized builds.*

## Known Limitations

- **Single Layer**: Assumes 2D planar environment; no support for multi-level structures
- **Frame Assumptions**: Graph position z-coordinate always 0 (costmap frame)
- **Update Strategy**: Recomputes entire graph on each costmap update (background threads not used)

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Write tests for new functionality
4. Ensure tests pass: `colcon test --packages-select stag`
5. Commit changes with clear messages
6. Push to branch and create Pull Request

## Citation

If you use Terrain Graph in your research, please cite:

```bibtex
@software{stag_2024,
  title = {STAG: Sparse Traversability-Aware Topological Graphs for Rover Navigation},
  author = {Gabriel},
  year = {2024},
  url = {<repository-url>}
}
```

## License

This project is licensed under the BSD 3-Clause License. See [LICENSE](../../../LICENSE) for details.

## Support & Feedback

- **Issues**: Report bugs and request features in this repository's GitHub Issues.
- **Discussions**: Use this repository's GitHub Discussions if enabled.
- **Documentation**: Package documentation is maintained in this README and the Docker README.

## Changelog

### v0.1.0 (2024)
- Initial release
- Multi-source graph extraction
- Docker support
- Comprehensive RViz visualization
- Unit test suite
