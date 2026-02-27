#!/bin/bash

echo "Installing RTAB-Map and dependencies..."
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y python3-opencv ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-launch

echo "Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build --symlink-install

echo "Starting Mapping Pipeline (RTAB-Map)"
source install/setup.bash

# Launch the SLAM pipeline
ros2 launch /workspace/ros2-depth-anything-v3-trt/drone_downward.launch.py
