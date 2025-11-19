from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    project_share = get_package_share_directory('project_launch')
    # try stack_master first (maps installed there), fallback to planning_pkg
    try:
        stack_share = get_package_share_directory('stack_master')
    except Exception:
        stack_share = get_package_share_directory('planning_pkg')

    default_map = os.path.join(stack_share, 'maps', 'teras', 'teras.yaml')
    default_raceline = os.path.join(stack_share, 'maps', 'teras', 'raceline.csv')

    raceline_file = LaunchConfiguration('raceline_file', default=default_raceline)
    map_path = LaunchConfiguration('map_path', default=default_map)
    frame_id = LaunchConfiguration('frame_id', default='map')
    control_params = LaunchConfiguration('control_params', default=os.path.join(project_share, 'config', 'control_params.yaml'))
    localization_params = LaunchConfiguration('localization_params', default=os.path.join(project_share, 'config', 'localization_params.yaml'))

    declare_raceline = DeclareLaunchArgument('raceline_file', default_value=default_raceline)
    declare_map = DeclareLaunchArgument('map_path', default_value=default_map)
    declare_frame = DeclareLaunchArgument('frame_id', default_value='map')
    declare_ctrl = DeclareLaunchArgument('control_params', default_value=os.path.join(project_share, 'config', 'control_params.yaml'))
    declare_loc = DeclareLaunchArgument('localization_params', default_value=os.path.join(project_share, 'config', 'localization_params.yaml'))

    # include f1tenth simulator if available
    f1tenth_launch = None
    try:
        f1tenth_share = get_package_share_directory('f1tenth_gym_ros')
        f1tenth_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(f1tenth_share, 'launch', 'gym_bridge_launch.py')),
            launch_arguments={'map_path': map_path}.items()
        )
    except Exception:
        f1tenth_launch = None

    raceline_node = Node(
        package='planning_pkg',
        executable='raceline_server',  # confirm executable name
        name='raceline_server',
        output='screen',
        parameters=[{'raceline_file': raceline_file, 'frame_id': frame_id, 'publish_vref': True}]
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

    nodes = [declare_raceline, declare_map, declare_frame, declare_ctrl, declare_loc]
    if f1tenth_launch:
        nodes.append(f1tenth_launch)
    nodes.extend([raceline_node, estimator_node, nmpc_node])

    return LaunchDescription(nodes)
