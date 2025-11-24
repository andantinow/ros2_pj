from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='True for Simulation (Gym), False for Real Robot'
    )
    
    sim_mode = LaunchConfiguration('sim_mode')

    try:
        f1tenth_gym_pkg = get_package_share_directory('f1tenth_gym_ros')
        sim_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(f1tenth_gym_pkg, 'launch', 'gym_bridge_launch.py')
            ),
            condition=IfCondition(sim_mode)
        )
    except PackageNotFoundError:
        sim_bridge = None
        print("Warning: f1tenth_gym_ros package not found. Sim mode will fail.")

    vesc_config = os.path.join(
        get_package_share_directory('control_pkg'),
        'config',
        'vesc.yaml'
    )

    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        parameters=[vesc_config],
        condition=UnlessCondition(sim_mode)
    )

    ackermann_to_vesc = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        parameters=[vesc_config],
        condition=UnlessCondition(sim_mode)
    )

    launch_actions = [sim_mode_arg]
    if sim_bridge:
        launch_actions.append(sim_bridge)
    launch_actions.append(vesc_driver)
    launch_actions.append(ackermann_to_vesc)

    return LaunchDescription(launch_actions)
