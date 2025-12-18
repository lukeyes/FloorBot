import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    semantic_nav_dir = get_package_share_directory('semantic_nav')
    ekf_config_path = os.path.join(semantic_nav_dir, 'config', 'ekf.yaml')

    # 1. RealSense Camera (T265 + D435)
    # CRITICAL: We disable 'publish_odom_tf' so T265 doesn't fight the EKF
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': 'linear_interpolation',
            'enable_depth': 'true',
            'enable_color': 'true',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'true',
            # T265 SPECIFIC SETTINGS FOR FUSION
            'enable_pose': 'true',
            'publish_odom_tf': 'false'  # <--- MUST BE FALSE
        }.items()
    )

    # 2. iRobot Create 1 Driver
    # We disable TF here too.
    create_driver = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        parameters=[{
            'dev': '/dev/ttyUSB0',
            'baud': 57600,
            'latch_cmd_duration': 0.2,
            'publish_tf': False  # <--- MUST BE FALSE
        }]
    )

    # 3. Robot Localization (The EKF)
    # This node now owns the "odom -> base_link" transform
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odom_filtered')]
    )

    # 4. Joystick Teleop
    joy_node = Node(package='joy', executable='joy_node')
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[{'enable_button': 4, 'scale_linear': 0.4}],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # 5. Depth to LaserScan
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/camera/aligned_depth_to_color/camera_info')
        ],
        parameters=[{'output_frame': 'camera_link', 'range_max': 5.0}]
    )

    # 6. Static TF (Base -> Camera)
    # Adjust 0.2 (20cm forward) and 0.15 (15cm up) to your real mounting!
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 7. Semantic Nodes
    semantic_vision = TimerAction(
        period=5.0,
        actions=[Node(
            package='semantic_nav',
            executable='vision_processor.py',
            name='vision_processor'
        )]
    )

    temporal_mapper = TimerAction(
        period=5.0,
        actions=[Node(
            package='semantic_nav',
            executable='temporal_mapper.py',
            name='temporal_mapper'
        )]
    )

    return LaunchDescription([
        realsense_launch,
        create_driver,
        ekf_node,  # <--- Added EKF
        joy_node,
        teleop_node,
        depth_to_scan,
        tf_base_to_camera,
        semantic_vision,
        temporal_mapper
    ])