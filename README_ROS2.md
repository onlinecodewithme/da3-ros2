# ROS 2 Depth Anything V3 - Docker Usage

This guide explains how to run the `ros2-depth-anything-v3-trt` node using Docker.

## Prerequisites
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) must be installed and configured.

## Usage

1.  **Run the Container**
    Execute the helper script to launch the container with GPU access and the current directory mounted:
    ```bash
    ./run_docker.sh
    ```

2.  **Inside the Container**
    The current directory is mounted at `/workspace/ros2-depth-anything-v3-trt`.

    You may need to build the workspace if you want to use the local code/configs:
    ```bash
    colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    ```

    **Generate TensorRT Engine:**
    Before launching, you need to generate the TensorRT engine from the ONNX model (or download a pre-built one).
    ```bash
    ./generate_engines.sh
    ```

    **Launch the Node:**
    ```bash
    ros2 launch depth_anything_v3 depth_anything_v3.launch.py
    ```

## Visualization
Since the container shares the host network (`--net host`), you can run `rviz2` on your host machine to visualize the topics:
-   `/depth_anything_v3/depth`
-   `/depth_anything_v3/points`
