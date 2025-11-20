import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# [추가됨] XML 파일을 읽기 위한 모듈 임포트
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. 패키지 경로 가져오기 ---
    project_launch_pkg = get_package_share_directory('project_launch')
    planning_pkg_share = get_package_share_directory('planning_pkg')
    stack_master_pkg = get_package_share_directory('stack_master')
    f1tenth_gym_pkg = get_package_share_directory('f1tenth_gym_ros')
    opponent_pkg = get_package_share_directory('opponent_publisher_cpp')

    # --- 2. 파일 경로 설정 ---
    default_map_path = os.path.join(stack_master_pkg, 'maps', 'teras', 'teras.yaml')
    default_raceline_path = os.path.join(planning_pkg_share, 'data', 'raceline.csv')
    sim_config_path = os.path.join(stack_master_pkg, 'config', 'SIM', 'sim_params.yaml')

    default_ctrl_params = os.path.join(project_launch_pkg, 'config', 'control_params.yaml')
    default_loc_params = os.path.join(project_launch_pkg, 'config', 'localization_params.yaml')
    rviz_config_path = os.path.join(project_launch_pkg, 'config', 'rviz_config.rviz')

    # --- 3. Launch Arguments ---
    map_path = LaunchConfiguration('map_path')
    raceline_file = LaunchConfiguration('raceline_file')
    
    declare_map_path = DeclareLaunchArgument(
        'map_path', default_value=default_map_path, description='Path to map yaml'
    )
    declare_raceline = DeclareLaunchArgument(
        'raceline_file', default_value=default_raceline_path, description='Path to raceline csv'
    )

    # --- 4. 노드 실행 ---

    # A. 시뮬레이터 실행 (Python 파일이므로 PythonLaunchDescriptionSource 사용)
    f1tenth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_gym_pkg, 'launch', 'gym_bridge_launch.py')
        ),
        launch_arguments={
            'map_yaml_path': map_path,
            'params_file': sim_config_path
        }.items()
    )

    # B. 상대차(주황색) 실행 (XML 파일이므로 XMLLaunchDescriptionSource 사용!)
    opponent_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource( # <--- 여기를 수정했습니다!
            os.path.join(opponent_pkg, 'launch', 'opponent_publisher_launch.xml')
        ),
        launch_arguments={
            'map_name': 'teras', 
            'speed': '0.5'
        }.items()
    )

    # C. 자차(파란색) - 우리 노드들 실행
    raceline_node = Node(
        package='planning_pkg',
        executable='raceline_server',
        name='raceline_server',
        output='screen',
        parameters=[{
            'raceline_file': raceline_file,
            'frame_id': 'map',
            'publish_vref': True
        }]
    )
    
    estimator_node = Node(
        package='localization_pkg',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[default_loc_params]
    )
    
    nmpc_node = Node(
        package='control_pkg',
        executable='simple_controller_node',
        name='nmpc_engine_node',
        output='screen',
        parameters=[default_ctrl_params],
        remappings=[('/drive', '/sim/drive')]
    )

    # D. RViz 실행
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        declare_map_path,
        declare_raceline,
        f1tenth_launch,
        opponent_launch,
        raceline_node,
        estimator_node,
        nmpc_node,
        rviz_node
    ])
