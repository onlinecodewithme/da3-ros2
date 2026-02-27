import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Camera Node (SimpleCam)
    # Reusing the simple_cam.py script
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
    # Using camera_link as the base frame for now
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # TF Camera Link -> Optical Link (Standard ROS camera orientation to Optical frame)
    tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_link', 'camera_link_optical']
    )

    # 4. RTAB-Map (Odometry + SLAM)
    # Using the standard rtabmap.launch.py from the package
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start', # Start fresh every time
            'rgb_topic': '/image_raw',
            'depth_topic': '/depth_anything_v3/depth',
            'camera_info_topic': '/camera_info',
            'frame_id': 'camera_link', # Robot Frame
            'approx_sync': 'true',
            'visual_odometry': 'true', # Enable Visual Odometry
            'queue_size': '20',
            'qos': '1', # Best Effort might be needed if topics are Best Effort? 
                        # SimpleCam is publishing Reliable? Let's check. Default is Reliable 10.
                        # Depth node subscribes with SensorData (Best Effort).
                        # Let's stick to default (Reliable) for RTAB-Map or 2 (SensorData)?
                        # RTAB-Map defaults to 2 (SensorData) for sub.
            'rviz': 'true' # Launch RTAB-Map's RViz
        }.items()
    )

    return LaunchDescription([
        camera_node,
        depth_node,
        tf_base_to_camera,
        tf_camera_to_optical,
        rtabmap_launch
    ])
