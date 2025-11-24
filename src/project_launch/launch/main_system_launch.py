import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    default_map_path = os.path.join(
        get_package_share_directory('localization_pkg'),
        'maps',
        'levine_2nd.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path to map YAML file'
    )

    sim_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',
        description='Enable simulation mode'
    )
    stack_master_pkg = get_package_share_directory('stack_master')
    localization_pkg = get_package_share_directory('localization_pkg')
    control_pkg = get_package_share_directory('control_pkg')
    planning_pkg = get_package_share_directory('planning_pkg')

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'bridge_launch.py')
        ),
        launch_arguments={'sim_mode': LaunchConfiguration('sim_mode')}.items()
    )

    mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stack_master_pkg, 'launch', 'mux_launch.py')
        )
    )

    lifecycle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stack_master_pkg, 'launch', 'lifecycle_mgr_launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('sim_mode')}.items()
    )

    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'map_launch.py')
        ),
        launch_arguments={
            'map_file': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('sim_mode')
        }.items()
    )
    
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'amcl_launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('sim_mode')}.items()
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_pkg, 'launch', 'planner_launch.py')
        )
    )

    
    return LaunchDescription([
        map_arg,
        sim_arg,
        control_launch,     
        mux_launch,         
        map_server_launch,  
        amcl_launch,        
        lifecycle_launch,   
        planning_launch    
    ])
