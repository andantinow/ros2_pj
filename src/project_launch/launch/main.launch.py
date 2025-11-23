import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # package share directories
    project_launch_pkg = get_package_share_directory('project_launch')
    planning_pkg_share = get_package_share_directory('planning_pkg')
    stack_master_pkg = get_package_share_directory('stack_master')
    f1tenth_gym_pkg = get_package_share_directory('f1tenth_gym_ros')
    opponent_pkg = get_package_share_directory('opponent_publisher_cpp')

    # launch arguments
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='teras',
        description='Map name (e.g., teras, silverstone).'
    )

    declare_raceline = DeclareLaunchArgument(
        'raceline_file',
        default_value=os.path.join(planning_pkg_share, 'data', 'raceline.csv'),
        description='Path to raceline csv'
    )

    # Option to avoid launching RViz here if included launches already start RViz
    declare_start_rviz = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start rviz from this launch (set false if included launches start rviz).'
    )

    # LaunchConfiguration shortcuts
    map_name_conf = LaunchConfiguration('map_name')
    raceline_file_conf = LaunchConfiguration('raceline_file')
    start_rviz_conf = LaunchConfiguration('start_rviz')

    # Build map yaml path: <stack_master_pkg>/maps/<map_name>/<map_name>.yaml
    # PythonExpression concatenates the LaunchConfiguration value with '.yaml' at runtime.
    map_yaml_path = PathJoinSubstitution([
        stack_master_pkg,
        'maps',
        map_name_conf,
        PythonExpression(["'", map_name_conf, "' + '.yaml'"])
    ])

    # Include other launches (example: gym bridge)
    f1tenth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_gym_pkg, 'launch', 'gym_bridge_launch.py')
        ),
        launch_arguments={
            # many gym bridges expect a map yaml path; pass computed path
            'map_yaml_path': map_yaml_path,
            'params_file': os.path.join(stack_master_pkg, 'config', 'SIM', 'sim_params.yaml')
        }.items()
    )

    opponent_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(opponent_pkg, 'launch', 'opponent_publisher_launch.xml')
        ),
        launch_arguments={
            'map_name': map_name_conf,
            'speed': '0.5'
        }.items()
    )

    # Nodes defined in this workspace
    raceline_node = Node(
        package='planning_pkg',
        executable='raceline_server',
        name='raceline_server',
        output='screen',
        parameters=[{
            'raceline_file': raceline_file_conf,
            'frame_id': 'map',
            'publish_vref': True
        }]
    )

    estimator_node = Node(
        package='localization_pkg',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[os.path.join(project_launch_pkg, 'config', 'localization_params.yaml')]
    )

    nmpc_node = Node(
        package='control_pkg',
        executable='simple_controller_node',
        name='nmpc_engine_node',
        output='screen',
        parameters=[os.path.join(project_launch_pkg, 'config', 'control_params.yaml')],
        remappings=[('/drive', '/sim/drive')]
    )

    # Optional: a small static transform to ensure 'map' exists immediately for RViz.
    # This is useful to avoid race conditions where RViz starts before any TF is published.
    # If another node already publishes the correct map->base_link transform, you can remove this node.
    static_map_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link'],
        output='screen'
    )

    # RViz node: start only if start_rviz == true
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(project_launch_pkg, 'config', 'rviz_config.rviz')],
        output='screen',
        condition=IfCondition(start_rviz_conf)
    )

    ld = LaunchDescription()

    # add declarations
    ld.add_action(declare_map_name)
    ld.add_action(declare_raceline)
    ld.add_action(declare_start_rviz)

    # add included launches and nodes
    ld.add_action(f1tenth_launch)
    ld.add_action(opponent_launch)
    ld.add_action(raceline_node)
    ld.add_action(estimator_node)
    ld.add_action(nmpc_node)

    # add static TF broadcaster (optional) before RViz to reduce race-condition errors
    ld.add_action(static_map_broadcaster)

    # add rviz (conditionally)
    ld.add_action(rviz_node)

    return ld
