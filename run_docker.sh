#!/bin/bash
xhost +local:root

# Run Docker container with GPU support, host networking, mounted volume, and device mapping
docker run -d --rm \
    --net host \
    --gpus all \
    --privileged \
    --group-add video \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v $(pwd):/workspace/ros2-depth-anything-v3-trt \
    -w /workspace/ros2-depth-anything-v3-trt \
    --name da3_demo \
    tillbeemelmanns/ros2-depth-anything-v3:latest-dev \
    tail -f /dev/null
