# Depth Anything V3 TensorRT ROS 2 Node


https://github.com/user-attachments/assets/d119d3b8-bba1-43a3-9f86-75db24e01235


A ROS 2 node for Depth Anything V3 depth estimation using TensorRT for real-time inference. This node subscribes to camera image and camera info topics and publishes directly both, a metric depth image and `PointCloud2` point cloud.

<!-- omit from toc -->
## Overview
- [Features](#features)
- [Dependencies](#dependencies)
- [Topics](#topics)
- [Parameters](#parameters)
- [Usage](#usage)
- [DA3-Based SLAM System](#da3-based-slam-system)
- [Model Preparation](#model-preparation)
- [Docker Image](#docker-image)
- [Building](#building)
- [Performance](#performance)
- [Architecture](#architecture)
- [Depth Postprocessing Pipeline](#depth-postprocessing-pipeline)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Citation](#citation)
- [Acknowledgements](#acknowledgements)



## Features

- **Real-time metric depth estimation** using Depth Anything V3 with TensorRT acceleration
- **Point cloud generation** from metric depth image
- **Debug visualization** with colormap options
- **Configurable precision** (FP16/FP32)

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> We cover a wide variety of research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your automated driving or robotics efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***


## Dependencies
- Tested with image `nvcr.io/nvidia/tensorrt:25.08-py3`
  - Ubuntu 24.04, ROS 2 Jazzy
  - CUDA 13
  - TensorRT 10.9
  
- Tested with image `nvcr.io/nvidia/tensorrt:25.03-py3`
  - Ubuntu 24.04, ROS 2 Jazzy
  - CUDA 12.8.1
  - TensorRT 10.9

Depending on your driver and CUDA version you need to select the appropriate base image.

## Topics

### Subscribed Topics

- `~/input/image` (sensor_msgs/Image): Input camera image
- `~/input/camera_info` (sensor_msgs/CameraInfo): Camera calibration info

### Published Topics  

- `~/output/depth_image` (sensor_msgs/Image): Depth image (32FC1 format)
- `~/output/point_cloud` (sensor_msgs/PointCloud2): Generated point cloud
- `~/output/depth_image_debug` (sensor_msgs/Image): Debug depth visualization (if enabled)

## Parameters

All parameters can be configured via `config/depth_anything_v3.param.yaml` or passed at launch time.

### Model Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `onnx_path` | string | `"models/DA3METRIC-LARGE.onnx"` | Path to Depth Anything V3 ONNX or TensorRT engine file |
| `precision` | string | `"fp16"` | Inference precision (`"fp16"` or `"fp32"`) |

### Sky Handling

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sky_threshold` | double | `0.3` | Threshold for sky classification from model's sky output (lower = more sky detected) |
| `sky_depth_cap` | double | `200.0` | Maximum depth value (meters) to assign to sky pixels |

### Point Cloud Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `point_cloud_downsample_factor` | int | `2` | Publish every Nth point (1 = full resolution, 10 = every 10th point) |
| `colorize_point_cloud` | bool | `true` | Add RGB colors from input image to point cloud |

### Debug Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_debug` | bool | `false` | Enable debug visualization output |
| `debug_colormap` | string | `"JET"` | Colormap for depth visualization (see below) |
| `debug_filepath` | string | `"/tmp/depth_anything_v3_debug/"` | Directory to save debug images |
| `write_colormap` | bool | `false` | Save colorized debug images to disk |
| `debug_colormap_min_depth` | double | `0.0` | Minimum depth value for colormap normalization (meters) |
| `debug_colormap_max_depth` | double | `50.0` | Maximum depth value for colormap normalization (meters) |

#### Available Colormaps
`JET`, `HOT`, `COOL`, `SPRING`, `SUMMER`, `AUTUMN`, `WINTER`, `BONE`, `GRAY`, `HSV`, `PARULA`, `PLASMA`, `INFERNO`, `VIRIDIS`, `MAGMA`, `CIVIDIS`

## Usage

### Basic Launch

```bash
ros2 launch depth_anything_v3 depth_anything_v3.launch.py
```

### With Custom Topics

```bash
ros2 launch depth_anything_v3 depth_anything_v3.launch.py \
    input_image_topic:=/your_camera/image_raw \
    input_camera_info_topic:=/your_camera/camera_info \
    output_depth_topic:=/depth_anything_v3/depth \
    output_point_cloud_topic:=/depth_anything_v3/points
```

### With Debug Enabled

```bash
ros2 launch depth_anything_v3 depth_anything_v3.launch.py \
    params_file:=src/depth_anything_v3/config/debug.param.yaml
```

## DA3-Based SLAM System

This repository includes an integrated Visual SLAM (Simultaneous Localization and Mapping) pipeline out-of-the-box, leveraging the metric depth generated by Depth Anything V3 and [RTAB-Map](http://introlab.github.io/rtabmap/).

The SLAM system uses a simulated or real monocam (via `simple_cam.py`), infers real-time metric depth using the DA3 TensorRT node, and feeds both the RGB image and depth map into RTAB-Map to compute Visual Odometry (VO) and construct a dense 3D point cloud map of the environment.

### Getting Started with SLAM

To easily install the required SLAM dependencies, build the workspace, and launch the entire pipeline, we provide a single convenience script: `start_mapping.sh`.

#### 1. Quick Start

Run the mapping script directly from the workspace root (ensure you are inside the Docker container or have ROS 2 Jazzy sourced):

```bash
# Sudo privileges might be required for apt installs within the script
sudo ./start_mapping.sh
```

#### 2. What `start_mapping.sh` Does

1. **Installs System Dependencies:** Uses `apt-get` to install `python3-opencv`, `ros-jazzy-rmw-cyclonedds-cpp`, and the RTAB-Map ROS 2 packages (`ros-jazzy-rtabmap-ros`, `ros-jazzy-rtabmap-launch`).
2. **Builds the Workspace:** Runs `colcon build --symlink-install` to ensure all custom nodes, Python scripts, and launch files are properly built and symlinked.
3. **Launches the Pipeline:** Executes `ros2 launch /workspace/ros2-depth-anything-v3-trt/drone_downward.launch.py` to start the entire system.

#### 3. SLAM Pipeline Architecture (`drone_downward.launch.py`)

The launch file orchestrates the following nodes simultaneously:

- **Camera Node (`simple_cam.py`):** Publishes RGB images to `/image_raw` and camera intrinsics to `/camera_info`.
- **Depth Anything V3 Node (`depth_anything_v3_main`):** Subscribes to the raw image, performs fast TensorRT inference using the DA3 metric model, and publishes the scale-aware metric depth to `/depth_anything_v3/depth`.
- **TF2 Broadcasters:** Establishes the spatial relationships in the TF tree, specifically connecting `base_link` -> `camera_link` (configured for a 45-degree downward tilt, mimicking a drone inspection camera) and creating the optical frame.
- **RTAB-Map Pipeline:** Takes the synchronized RGB and Depth topics to compute Visual Odometry (VO) frame-by-frame and aggregate the data into a globally consistent 3D map. It is configured with parameters to filter out ground planes (via `MaxGroundHeight` and `MinGroundHeight`) and optimize feature tracking.
- **RViz2:** Opens the visualization interface using `rviz_config.rviz`.

#### 4. Visualization

When the mapping script runs, RViz2 will automatically open. You will be able to visualize:
- **Live Camera Feed**: The raw 2D image from the camera.
- **Depth Map**: The DA3-estimated metric depth output.
- **Odometry Trajectory**: The calculated path of the camera through space.
- **3D Map**: The dense point cloud map being incrementally built by RTAB-Map.

## Model Preparation

1. **Obtain the ONNX model (Two Options)**:
   A.  Download the ONNX file from Huggingface: [https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX](https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX)
   B.  Generate ONNX following the instruction [here](onnx/README.md)
2. **Place model file**: Put the ONNX/engine file in the `models/` directory
3. **Update configuration**: Modify `config/depth_anything_v3.param.yaml` with the correct model path

Expected model input format:
- Input shape: [1, 3, 280, 504] (batch, channels, height, width)
- Input type: float32
- Value range: [0, 1] (normalized with ImageNet mean/std)

Expected model outputs:
- `depth`: [1, 1, 280, 504] - metric depth map (float32)
- `sky`: [1, 1, 280, 504] - sky classification logits (float32, lower values = sky)


## Docker Image
We precompile and provide a Docker image with all dependencies installed and ready 
to use. You can pull it from Docker Hub:
```bash
docker pull tillbeemelmanns/ros2-depth-anything-v3:latest-dev
```

If you want to run rosbags and visualize the output in rviz2 make sure to install  it
```bash
apt update && apt install -y \
ros-jazzy-rviz2 \
ros-jazzy-rosbag2 \
ros-jazzy-rosbag2-storage-mcap
```

We recommend to use the docker image in combination with our other tools for Docker and ROS.
- [*docker-ros*](https://github.com/ika-rwth-aachen/docker-ros) automatically builds minimal container images of ROS applications <a href="https://github.com/ika-rwth-aachen/docker-ros"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/docker-ros?style=social"/></a>
- [*docker-run*](https://github.com/ika-rwth-aachen/docker-run) is a CLI tool for simplified interaction with Docker images <a href="https://github.com/ika-rwth-aachen/docker-run"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/docker-run?style=social"/></a>



## Building

```bash
# From your ROS 2 workspace
colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# Optional: Pre-build TensorRT engines
./generate_engines.sh
```

## Performance
The node is optimized for real-time performance.

Performance on Quadro RTX 6000:
- **DA3METRIC-LARGE**: 50 FPS

## Architecture

```
Input Image + Camera Info
         ↓
    Preprocessing (CPU/GPU)
         ↓  
    TensorRT Inference (GPU)
         ↓
    Postprocessing (CPU)
         ↓
   Depth Image + Point Cloud
```

## Depth Postprocessing Pipeline

Depth Anything V3 predicts a dense metric depth map. After TensorRT inference, the node performs the following postprocessing steps:

### 1. Depth Extraction
- The single-channel NCHW tensor is reshaped into a `cv::Mat`
- Negative depth values are clamped to zero

### 2. Focal Length Scaling
- The raw depth output is scaled based on the camera's focal length
- Scale factor: `focal_pixels / 300.0` where `focal_pixels = (fx + fy) / 2`
- This aligns the model's internal focal normalization with your actual camera intrinsics

### 3. Sky Handling
The model outputs a separate sky classification tensor:
- Pixels with sky confidence below `sky_threshold` are classified as sky
- Sky pixels are filled with a depth value derived from the 99th percentile of valid (non-sky) depths
- The fill value is capped at `sky_depth_cap` meters to avoid unrealistic far distances

### 4. Resolution Upscaling
- The depth map is resized from model resolution back to the original camera resolution using cubic interpolation

### 5. Point Cloud Generation
For each pixel in the depth map:
```
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```
Where `(cx, cy)` is the principal point and `(fx, fy)` are the focal lengths from `CameraInfo`.

Optional features:
- **Downsampling**: Use `point_cloud_downsample_factor` to reduce point count for visualization or bandwidth
- **Colorization**: When `colorize_point_cloud` is enabled, RGB values from the input image are added to each 3D point


## Troubleshooting

### Common Issues

1. **TensorRT engine building fails**:
   - Check CUDA/TensorRT compatibility
   - Verify ONNX model format
   - Increase workspace size

2. **Point cloud appears incorrect**:
   - Verify camera_info calibration
   - Check coordinate frame conventions
   - Validate depth value units

3. **Performance issues**:
   - Enable FP16 precision
   - Check GPU memory usage

### Debug Mode

Enable debug mode to troubleshoot:
```yaml
enable_debug: true
debug_colormap: "JET"
debug_colormap_min_depth: 0.0
debug_colormap_max_depth: 50.0
write_colormap: true
```

This will publish colorized depth images and save them to disk for inspection.


## License

This project is licensed under the Apache License, Version 2.0 - see the [LICENSE](LICENSE) file for details.

## Citation
If you use this code in your research, please cite the following:

```bibtex
@misc{beemelmanns2024depth,
  author = {Till Beemelmanns},
  title = {ros2-depth-anything-v3-trt: ROS2 TensorRT Node for Monocular Metric Depth estimation and Point Cloud generation with Depth Anything V3},
  year = {2025},
  publisher = {GitHub},
  url = {https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt}
}
```

## Acknowledgements
Thanks to the following repositories for inspiration:

- [ROS 2 MoGE TRT Node](https://github.com/ika-rwth-aachen/ros2-moge-trt/tree/main)
- [Depth-Anything-V3](https://github.com/ByteDance-Seed/depth-anything-3)
- [Monocular_Depth_Estimation_TRT](https://github.com/yester31/Monocular_Depth_Estimation_TRT)
- [DepthAnything-ROS](https://github.com/scepter914/DepthAnything-ROS)
