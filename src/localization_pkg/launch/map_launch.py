import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            get_package_share_directory('localization_pkg'),
            'maps',
            'levine_2nd.yaml'
        ),
        description='Full path to map yaml file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
