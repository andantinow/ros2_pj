import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    raceline_file_arg = DeclareLaunchArgument(
        'raceline_file',
        default_value='',
        description='Path to raceline CSV file (relative to package share or absolute)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the published path'
    )
    
    publish_vref_arg = DeclareLaunchArgument(
        'publish_vref',
        default_value='true',
        description='Whether to publish velocity reference array'
    )
    
    enable_smoothing_arg = DeclareLaunchArgument(
        'enable_smoothing',
        default_value='false',
        description='Enable path smoothing'
    )
    
    smoothing_window_arg = DeclareLaunchArgument(
        'smoothing_window',
        default_value='5',
        description='Smoothing window size (must be odd, >= 3)'
    )
    
    resample_spacing_arg = DeclareLaunchArgument(
        'resample_spacing',
        default_value='0.0',
        description='Resample path with this spacing (0.0 = no resampling)'
    )
    
    # Raceline server node
    raceline_server_node = Node(
        package='planning_pkg',
        executable='raceline_server',
        name='raceline_server',
        output='screen',
        parameters=[{
            'raceline_file': LaunchConfiguration('raceline_file'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_vref': LaunchConfiguration('publish_vref'),
            'enable_smoothing': LaunchConfiguration('enable_smoothing'),
            'smoothing_window': LaunchConfiguration('smoothing_window'),
            'resample_spacing': LaunchConfiguration('resample_spacing'),
        }]
    )
    
    # Optional: Simple path planner node
    simple_planner_node = Node(
        package='planning_pkg',
        executable='simple_path_planner',
        name='simple_path_planner',
        output='screen',
        parameters=[{
            'odom_topic': '/car_state/odom_GT',
            'path_topic': '/planning/path',
            'frame_id': 'map',
            'path_spacing': 0.1,
            'enable_smoothing': True,
            'smoothing_window': 5,
            'publish_fixed_path': True,
            'publish_rate': 1.0,
        }]
    )
    
    return LaunchDescription([
        raceline_file_arg,
        frame_id_arg,
        publish_vref_arg,
        enable_smoothing_arg,
        smoothing_window_arg,
        resample_spacing_arg,
        raceline_server_node,
        # Uncomment to enable simple path planner:
        # simple_planner_node,
    ])
