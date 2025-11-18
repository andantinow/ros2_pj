from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    raceline_file = LaunchConfiguration('raceline_file')
    frame_id = LaunchConfiguration('frame_id')
    control_params = LaunchConfiguration('control_params')
    localization_params = LaunchConfiguration('localization_params')

    declare_raceline = DeclareLaunchArgument('raceline_file', default_value='data/raceline.csv')
    declare_frame = DeclareLaunchArgument('frame_id', default_value='map')
    declare_ctrl = DeclareLaunchArgument(
        'control_params',
        default_value=os.path.join(get_package_share_directory('project_launch'), 'config', 'control_params.yaml')
    )
    declare_loc = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(get_package_share_directory('project_launch'), 'config', 'localization_params.yaml')
    )

    raceline_node = Node(
        package='planning_pkg',
        executable='raceline_server',
        name='raceline_server',
        output='screen',
        parameters=[{'raceline_file': raceline_file,'frame_id': frame_id,'publish_vref': True}]
    )
    estimator_node = Node(
        package='localization_pkg',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[localization_params]
    )
    nmpc_node = Node(
        package='control_pkg',
        executable='nmpc_engine_node',
        name='nmpc_engine_node',
        output='screen',
        parameters=[control_params]
    )

    return LaunchDescription([
        declare_raceline,
        declare_frame,
        declare_ctrl,
        declare_loc,
        raceline_node,
        estimator_node,
        nmpc_node
    ])
