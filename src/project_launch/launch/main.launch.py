# ~/ros2_ws/src/project_launch/launch/main.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Localization Node 실행 (위치 발행)
        Node(
            package='localization_pkg',
            executable='pose_publisher',
            name='loc_publisher',
            output='screen'
        ),
        # 2. Planning Node 실행 (경로 계획/발행)
        Node(
            package='planning_pkg',
            executable='path_planner',
            name='planner_node',
            output='screen'
        ),
        # 3. Control Node 실행 (제어 명령 발행)
        Node(
            package='control_pkg',
            executable='controller',
            name='controller_node',
            output='screen'
        )
    ])
