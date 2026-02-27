import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Simple Python Camera Node
        ExecuteProcess(
            cmd=['python3', '/workspace/ros2-depth-anything-v3-trt/simple_cam.py'],
            output='screen'
        ),

        # Depth Anything Node
        Node(
            package='depth_anything_v3',
            executable='depth_anything_v3_main',
            name='depth_anything_v3_node',
            output='screen',
            parameters=[
                '/workspace/ros2-depth-anything-v3-trt/depth_anything_v3/config/depth_anything_v3.param.yaml',
                {'onnx_path': '/workspace/ros2-depth-anything-v3-trt/depth_anything_v3/models/DA3METRIC-LARGE.fp16-batch1.engine'}
            ],
            remappings=[
                ('~/input/image', '/image_raw'),
                ('~/input/camera_info', '/camera_info'),
                ('~/output/depth_image', '/depth_anything_v3/depth'),
                ('~/output/point_cloud', '/depth_anything_v3/points'),
                ('~/output/debug_image', '/depth_anything_v3/debug_image')
            ]
        ),

        # Static Transform Publisher (camera_link -> camera_link_optical)
        # Assuming camera_link is the robot frame, optical is for image
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'camera_link', 'camera_link_optical']
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/workspace/ros2-depth-anything-v3-trt/rviz_config.rviz'],
            output='screen'
        )
    ])
