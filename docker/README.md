# Docker Runtime

Build the controlled ROS environment:

```bash
docker compose build
```

Run the STAG node:

```bash
docker compose up stag
```

By default, Compose uses:

```text
ROS_DOMAIN_ID=42
```

Override it when needed:

```bash
ROS_DOMAIN_ID=7 docker compose up stag
```

The default container command launches:

```bash
ros2 launch stag stag.launch.py \
  params_file:=/config/stag.yaml \
  graph_save_path:=/graph_output/stag_latest.npz
```

The mounted config comes from:

```text
ros_ws/src/stag/config/stag.yaml
```

Saved graph snapshots are written on the host under:

```text
docker/graph_output/
```
