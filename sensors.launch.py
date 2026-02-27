import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # 1. Camera Node (SimpleCam)
    camera_node = ExecuteProcess(
        cmd=['python3', '/workspace/ros2-depth-anything-v3-trt/simple_cam.py'],
        output='screen'
    )

    # 2. Depth Estimation Node
    depth_node = Node(
        package='depth_anything_v3',
        executable='depth_anything_v3_main',
        name='depth_anything_v3',
        output='screen',
        parameters=[{
            'onnx_path': '/workspace/ros2-depth-anything-v3-trt/depth_anything_v3/models/DA3METRIC-LARGE.fp16-batch1.engine',
            'input_topic': '/image_raw',
            'output_depth_topic': '/depth_anything_v3/depth',
            'output_points_topic': '/depth_anything_v3/points'
        }],
        remappings=[
            ('~/input/image', '/image_raw'),
            ('~/input/camera_info', '/camera_info'),
            ('~/output/depth_image', '/depth_anything_v3/depth'),
            ('~/output/point_cloud', '/depth_anything_v3/points'),
            ('~/output/debug_image', '/depth_anything_v3/debug_image')
        ]
    )

    # 3. Static TF (Base Link -> Camera Link)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # TF Camera Link -> Optical Link
    tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_link', 'camera_link_optical']
    )

    return LaunchDescription([
        camera_node,
        depth_node,
        tf_base_to_camera,
        tf_camera_to_optical
    ])
