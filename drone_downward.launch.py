import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

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

    # 3. TF: Drone Body -> Camera (45 deg tilt down)
    # x y z yaw pitch roll
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0', '0', '0.785', '0', 'base_link', 'camera_link']
    )
    
    # TF: Camera -> Optical (Standard)
    tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_link', 'camera_link_optical']
    )

    # 4. RTAB-Map
    # Using 'rtabmap_launch' package
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --Grid/FromDepth true --Grid/MaxGroundHeight 0.0 --Grid/MinGroundHeight -10.0 --Grid/NormalsSegmentation false --Vis/MinInliers 10 --GFTT/MinDistance 10',
            'rgb_topic': '/image_raw',
            'depth_topic': '/depth_anything_v3/depth',
            'camera_info_topic': '/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'visual_odometry': 'true',
            'queue_size': '20',
            'qos': '1',
            'rviz': 'false', # We launch our own RViz
        }.items()
    )

    # 5. Custom RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/workspace/ros2-depth-anything-v3-trt/rviz_config.rviz'],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        depth_node,
        tf_base_to_camera,
        tf_camera_to_optical,
        rtabmap_launch,
        rviz_node
    ])
