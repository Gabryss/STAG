FROM ros:jazzy-ros-base

ENV ROS_DISTRO=jazzy \
    ROS_WS=/stag_ws \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-scipy \
    python3-skimage \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}

COPY ros_ws/src/stag src/stag

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --packages-select stag

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "stag", "stag.launch.py"]
