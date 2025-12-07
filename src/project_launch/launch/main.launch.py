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
    localization_pkg = get_package_share_directory('localization_pkg')

    # launch arguments
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='small_hall_orig',
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
    declare_bridge_start_rviz = DeclareLaunchArgument(
        'bridge_start_rviz',
        default_value='true',
        description='Whether to also start RViz inside the f1tenth_gym bridge launch.'
    )
    
    # Controller selection: 'nmpc' or 'simple'
    declare_controller = DeclareLaunchArgument(
        'controller',
        default_value='nmpc',  # Default to NMPC controller for better performance
        description='Controller to use: nmpc or simple (Pure Pursuit + PID + Stanley)'
    )
    
    # Opponent line mode: 'outer' or 'inner'
    declare_opponent_line_mode = DeclareLaunchArgument(
        'opponent_line_mode',
        default_value='outer',
        description='Opponent raceline mode: outer (runs on outside) or inner (runs on inside)'
    )
    
    # Opponent speed factor (fraction of ego speed)
    declare_opponent_speed = DeclareLaunchArgument(
        'opponent_speed',
        default_value='3.25',
        description='Opponent target speed in m/s (should be ~50% of ego max speed)'
    )

    # LaunchConfiguration shortcuts
    map_name_conf = LaunchConfiguration('map_name')
    raceline_file_conf = LaunchConfiguration('raceline_file')
    start_rviz_conf = LaunchConfiguration('start_rviz')
    bridge_start_rviz_conf = LaunchConfiguration('bridge_start_rviz')
    controller_conf = LaunchConfiguration('controller')
    opponent_line_mode_conf = LaunchConfiguration('opponent_line_mode')
    opponent_speed_conf = LaunchConfiguration('opponent_speed')

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
            'params_file': os.path.join(stack_master_pkg, 'config', 'SIM', 'sim_params.yaml'),
            'start_rviz': bridge_start_rviz_conf
        }.items()
    )

    opponent_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(opponent_pkg, 'launch', 'opponent_publisher_launch.xml')
        ),
        launch_arguments={
            'map_name': map_name_conf,
            'speed': opponent_speed_conf,
            'path_topic': '/opponent_raceline'
        }.items()
    )

    # Nodes defined in this workspace
    # Main raceline server for ego car (global optimal racing line)
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
    
    # Opponent raceline server - publishes opponent raceline based on line_mode
    # - 'outer': opponent runs on outer side of track (default)
    # - 'inner': opponent runs on inner side of track
    # This creates predictable opponent behavior and clear overtaking corridors
    opponent_raceline_file = PathJoinSubstitution([
        planning_pkg_share,
        'data',
        PythonExpression(["'opponent_' + '", opponent_line_mode_conf, "' + '.csv'"])
    ])
    
    opponent_raceline_node = Node(
        package='planning_pkg',
        executable='raceline_server',
        name='opponent_raceline_server',
        output='screen',
        parameters=[{
            'raceline_file': opponent_raceline_file,
            'frame_id': 'map',
            'publish_vref': False,
            'path_topic': '/opponent_raceline',
            'vref_topic': '/opponent_vref'
        }]
    )

    estimator_node = Node(
        package='localization_pkg',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[os.path.join(project_launch_pkg, 'config', 'localization_params.yaml')]
    )

    # Scan Processor Node - LiDAR scan processing for localization
    scan_processor_node = Node(
        package='localization_pkg',
        executable='scan_processor_node',
        name='scan_processor_node',
        output='screen',
        parameters=[os.path.join(localization_pkg, 'config', 'dual_ekf_config.yaml')]
    )

    # Dual EKF Node - Two-stage EKF for robust state estimation
    dual_ekf_node = Node(
        package='localization_pkg',
        executable='dual_ekf_node',
        name='dual_ekf_node',
        output='screen',
        parameters=[os.path.join(localization_pkg, 'config', 'dual_ekf_config.yaml')]
    )

    # Remove static transform - let odometry/gym_bridge publish the actual transform
    # static_map_broadcaster = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_base_link_broadcaster',
    #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link'],
    #     output='screen'
    # )

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
    ld.add_action(declare_bridge_start_rviz)
    ld.add_action(declare_controller)
    ld.add_action(declare_opponent_line_mode)
    ld.add_action(declare_opponent_speed)

    # add included launches and nodes
    ld.add_action(f1tenth_launch)
    ld.add_action(opponent_launch)
    ld.add_action(raceline_node)
    ld.add_action(opponent_raceline_node)
    ld.add_action(estimator_node)
    
    # Controller selection: use simple controller by default, NMPC when specified
    # Both controllers are added with conditions
    from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
    
    # Simple controller (default)
    simple_controller_node_cond = Node(
        package='control_pkg',
        executable='simple_controller_node',
        name='simple_controller',
        output='screen',
        parameters=[os.path.join(project_launch_pkg, 'config', 'control_params.yaml')],
        condition=LaunchConfigurationNotEquals('controller', 'nmpc')
    )
    ld.add_action(simple_controller_node_cond)
    
    # NMPC controller (when controller:=nmpc)
    # Uses dedicated config file for easier tuning
    # See src/project_launch/config/nmpc_params.yaml for detailed parameter documentation
    nmpc_controller_node_cond = Node(
        package='control_pkg',
        executable='nmpc_engine_node',
        name='nmpc_engine_node',
        output='screen',
        parameters=[os.path.join(project_launch_pkg, 'config', 'nmpc_params.yaml')],
        condition=LaunchConfigurationEquals('controller', 'nmpc')
    )
    ld.add_action(nmpc_controller_node_cond)
    
    ld.add_action(scan_processor_node)
    ld.add_action(dual_ekf_node)

    # Static TF broadcaster removed - use actual odometry transform
    # ld.add_action(static_map_broadcaster)

    # add rviz (conditionally)
    ld.add_action(rviz_node)

    return ld
