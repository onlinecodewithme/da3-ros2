#!/bin/bash
set -e

echo "Installing USB Cam and RViz..."
export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y python3-opencv ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rtabmap-ros ros-jazzy-rviz2

echo "Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

echo "Launching All-in-One..."
ros2 launch all_in_one.launch.py
