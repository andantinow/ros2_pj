# ~/ros2_ws/src/project_launch/launch/main.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Localization Node (Publishes simulated Odometry to /localization/pose)
        Node(
            package='localization_pkg',
            executable='odom_subscriber',
            name='loc_publisher',
            output='screen'
        ),
        
        # 2. Planning Node (Publishes fixed path to /planning/path)
        Node(
            package='planning_pkg',
            executable='path_planner',
            name='planner_node',
            output='screen'
        ),
        
        # 3. Control Node (Subscribes to Pose/Path, Publishes Twist to /cmd_vel)
        Node(
            package='control_pkg',
            executable='controller',
            name='controller_node',
            output='screen'
        ),

        # 4. TF Static Broadcaster Node (Essential for Rviz visualization)
        # Publishes the fixed relationship between the 'world' frame and the 'base_link' frame.
        # Arguments: x, y, z, roll, pitch, yaw, parent_frame, child_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'] 
        )
    ])
