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

    # LaunchConfiguration shortcuts
    map_name_conf = LaunchConfiguration('map_name')
    raceline_file_conf = LaunchConfiguration('raceline_file')
    start_rviz_conf = LaunchConfiguration('start_rviz')
    bridge_start_rviz_conf = LaunchConfiguration('bridge_start_rviz')
    controller_conf = LaunchConfiguration('controller')

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

    # add included launches and nodes
    ld.add_action(f1tenth_launch)
    ld.add_action(opponent_launch)
    ld.add_action(raceline_node)
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
    nmpc_controller_node_cond = Node(
        package='control_pkg',
        executable='nmpc_engine_node',
        name='nmpc_controller',
        output='screen',
        parameters=[{
            'prediction_horizon': 1.0,
            'prediction_steps': 10,
            'control_rate_hz': 50.0,
            'nominal_speed': 2.0,
            'solver_wheelbase': 0.33,
            'use_dual_ekf': True,
            'odom_topic': '/dual_ekf/global_odom',
            'path_topic': '/global_raceline',
            'drive_topic': '/drive',
            'w_pos': 10.0,
            'w_yaw': 5.0,
            'w_vel': 2.0,
            'w_steer': 1.0,
            'w_accel': 0.5,
            'max_steer': 0.5,
            'max_speed': 5.0,
            'max_accel': 3.0,
            'min_accel': -5.0,
        }],
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
