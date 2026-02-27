#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build
echo "Starting build..."
colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source install
source install/setup.bash
echo "Build successful!"

# Instructions
echo "To run the node:"
echo "1. Ensure you have the ONNX model in 'models/'"
echo "2. Run './generate_engines.sh' to create the TensorRT engine"
echo "3. Run 'ros2 launch depth_anything_v3 depth_anything_v3.launch.py'"
