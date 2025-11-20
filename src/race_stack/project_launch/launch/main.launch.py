from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    project_launch_dir = get_package_share_directory('project_launch')
    
    raceline_file = LaunchConfiguration('raceline_file')
    control_params_file = LaunchConfiguration('control_params_file')
    localization_params_file = LaunchConfiguration('localization_params_file')
    
    declare_raceline_file = DeclareLaunchArgument(
        'raceline_file',
        default_value=os.path.join(project_launch_dir, 'config', 'raceline.csv'),
        description='Path to raceline CSV file')
    
    declare_control_params = DeclareLaunchArgument(
        'control_params_file',
        default_value=os.path.join(project_launch_dir, 'config', 'control_params.yaml'),
        description='Path to control parameters YAML')
    
    declare_localization_params = DeclareLaunchArgument(
        'localization_params_file',
        default_value=os.path.join(project_launch_dir, 'config', 'localization_params.yaml'),
        description='Path to localization parameters YAML')
    
    raceline_node = Node(
        package='planning_pkg',
        executable='raceline_server',
        name='raceline_server_node',
        parameters=[{
            'raceline_file': raceline_file,
            'frame_id': 'map',
            'publish_vref': True,
            'publish_kappa': True
        }],
        output='screen')
    
    estimator_node = Node(
        package='localization_pkg',
        executable='estimator_node',
        name='estimator_node',
        parameters=[localization_params_file],
        output='screen')
    
    nmpc_node = Node(
        package='control_pkg',
        executable='nmpc_engine_node',
        name='nmpc_engine_node',
        parameters=[control_params_file],
        output='screen')
    
    return LaunchDescription([
        declare_raceline_file,
        declare_control_params,
        declare_localization_params,
        raceline_node,
        estimator_node,
        nmpc_node
    ])

