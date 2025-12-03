"""
Dual EKF Launch File

This launch file starts the dual EKF localization system consisting of:
1. Scan Processor Node - Processes LiDAR scan data for feature extraction
2. Dual EKF Node - Two-stage EKF for robust state estimation

Usage:
  ros2 launch localization_pkg dual_ekf_launch.py
  
Parameters can be overridden via command line:
  ros2 launch localization_pkg dual_ekf_launch.py odom_topic:=/car_state/odom
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('localization_pkg')
    
    # Config file path
    config_file = os.path.join(pkg_share, 'config', 'dual_ekf_config.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Input odometry topic'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensors/imu/raw',
        description='Input IMU topic'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Input laser scan topic'
    )
    
    global_pose_topic_arg = DeclareLaunchArgument(
        'global_pose_topic',
        default_value='/amcl_pose',
        description='Global pose topic (from AMCL or particle filter)'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    # Scan Processor Node
    scan_processor_node = Node(
        package='localization_pkg',
        executable='scan_processor_node',
        name='scan_processor_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
            }
        ],
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')),
        ]
    )
    
    # Dual EKF Node
    dual_ekf_node = Node(
        package='localization_pkg',
        executable='dual_ekf_node',
        name='dual_ekf_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'global_pose_topic': LaunchConfiguration('global_pose_topic'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        imu_topic_arg,
        scan_topic_arg,
        global_pose_topic_arg,
        publish_tf_arg,
        scan_processor_node,
        dual_ekf_node,
    ])
