import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mux_config = os.path.join(
        get_package_share_directory('stack_master'),
        'config',
        'ackermann_mux.yaml'
    )

    return LaunchDescription([
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            output='screen',
            parameters=[mux_config],
            remappings=[
                ('ackermann_cmd_out', '/drive')
            ]
        )
    ])
