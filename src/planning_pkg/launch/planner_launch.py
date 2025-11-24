import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('planning_pkg'),
        'config',
        'pure_pursuit_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='planning_pkg',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[config],
            # 파라미터 파일의 drive_topic을 사용하지 않고 리매핑을 쓸 수도 있음
            # remappings=[('/drive', '/drive_nav')] 
        )
    ])
