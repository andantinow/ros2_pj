import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. 패키지 경로 가져오기 ---
    project_launch_pkg = get_package_share_directory('project_launch')
    planning_pkg_share = get_package_share_directory('planning_pkg')
    stack_master_pkg = get_package_share_directory('stack_master')
    f1tenth_gym_pkg = get_package_share_directory('f1tenth_gym_ros')
    opponent_pkg = get_package_share_directory('opponent_publisher_cpp')

    # --- 2. 기본 파일 경로 및 설정 ---
    # 맵 이름만 바꾸면 모든 경로가 자동 생성되도록 설계합니다.
    default_map_name = 'teras'
    default_raceline_path = os.path.join(planning_pkg_share, 'data', 'raceline.csv')
    sim_config_path = os.path.join(stack_master_pkg, 'config', 'SIM', 'sim_params.yaml')
    
    default_ctrl_params = os.path.join(project_launch_pkg, 'config', 'control_params.yaml')
    default_loc_params = os.path.join(project_launch_pkg, 'config', 'localization_params.yaml')
    rviz_config_path = os.path.join(project_launch_pkg, 'config', 'rviz_config.rviz')

    # --- 3. Launch Arguments 선언 ---
    
    # [핵심] map_name 하나로 통합 제어
    declare_map_name = DeclareLaunchArgument(
        'map_name', 
        default_value=default_map_name, 
        description='Map name (e.g., teras, silverstone). This sets both sim map and opponent map.'
    )

    declare_raceline = DeclareLaunchArgument(
        'raceline_file', 
        default_value=default_raceline_path, 
        description='Path to raceline csv'
    )

    # --- 4. 변수 설정 (Configuration & Substitution) ---
    
    # LaunchConfiguration으로 값을 받아옵니다.
    map_name_conf = LaunchConfiguration('map_name')
    raceline_file_conf = LaunchConfiguration('raceline_file')

    # [자동 경로 생성] map_name에 따라 .yaml 파일 경로를 동적으로 만듭니다.
    # 구조: stack_master/maps/{map_name}/{map_name}.yaml
    map_yaml_path = PathJoinSubstitution([
        stack_master_pkg,
        'maps',
        map_name_conf,
        PythonExpression(["'", map_name_conf, "' + '.yaml'"]) # 문자열 결합: name + .yaml
    ])

    # --- 5. 노드 및 런치 포함 실행 ---

    # A. 시뮬레이터 실행
    f1tenth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_gym_pkg, 'launch', 'gym_bridge_launch.py')
        ),
        launch_arguments={
            'map_yaml_path': map_yaml_path, # 위에서 만든 동적 경로 주입
            'params_file': sim_config_path
        }.items()
    )

    # B. 상대차(주황색) 실행
    opponent_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(opponent_pkg, 'launch', 'opponent_publisher_launch.xml')
        ),
        launch_arguments={
            'map_name': map_name_conf,  # [수정됨] 하드코딩 대신 변수 사용
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
        declare_map_name,
        declare_raceline,
        f1tenth_launch,
        opponent_launch,
        raceline_node,
        estimator_node,
        nmpc_node,
        rviz_node
    ])
