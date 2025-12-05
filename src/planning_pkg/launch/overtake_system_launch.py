"""
Launch file for the Opponent Detection and Overtake Planning System

This launch file starts:
1. opponent_detector - Detects leading vehicles using LiDAR + odom + raceline
2. global_overtake_planner - Plans overtake paths and manages state machine

Prerequisites:
- raceline_server must be running (publishing /global_raceline)
- /scan (LaserScan) and /odom (Odometry) topics must be available

Topics:
  Input:
    /scan - LiDAR data
    /odom - Vehicle odometry
    /global_raceline - Base raceline path
  Output:
    /opponent_info - Detected opponent in Frenet coordinates
    /opponent_marker - RViz visualization of opponent
    /global_overtake_raceline - Planned overtake path
    /active_raceline - Currently active path (raceline or overtake)
    /overtake_state - Current state machine state
    /overtake_markers - RViz visualization of overtake state

Usage:
  ros2 launch planning_pkg overtake_system_launch.py
  ros2 launch planning_pkg overtake_system_launch.py d_max:=0.8 trigger_distance:=2.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # =========================================================================
    # Opponent Detector Parameters
    # =========================================================================
    front_angle_min_arg = DeclareLaunchArgument(
        'front_angle_min',
        default_value='-0.26',  # -15 degrees
        description='Minimum angle for front sector detection (radians)'
    )
    
    front_angle_max_arg = DeclareLaunchArgument(
        'front_angle_max',
        default_value='0.26',  # +15 degrees
        description='Maximum angle for front sector detection (radians)'
    )
    
    max_detection_range_arg = DeclareLaunchArgument(
        'max_detection_range',
        default_value='10.0',
        description='Maximum LiDAR range to consider for opponent detection (m)'
    )
    
    lane_half_width_arg = DeclareLaunchArgument(
        'lane_half_width',
        default_value='0.6',
        description='Half lane width for opponent validation (m)'
    )
    
    min_ahead_margin_arg = DeclareLaunchArgument(
        'min_ahead_margin',
        default_value='0.3',
        description='Minimum s difference to consider as ahead (m)'
    )
    
    max_ahead_distance_arg = DeclareLaunchArgument(
        'max_ahead_distance',
        default_value='5.0',
        description='Maximum s difference to track opponent (m)'
    )
    
    # =========================================================================
    # Overtake Planner Parameters
    # =========================================================================
    d_max_arg = DeclareLaunchArgument(
        'd_max',
        default_value='0.6',
        description='Maximum lateral offset for overtake (m)'
    )
    
    s_buffer_start_arg = DeclareLaunchArgument(
        's_buffer_start',
        default_value='1.0',
        description='Start overtake this far ahead of current position (m)'
    )
    
    s_overlap_arg = DeclareLaunchArgument(
        's_overlap',
        default_value='1.5',
        description='Pass this far beyond opponent before returning (m)'
    )
    
    s_buffer_end_arg = DeclareLaunchArgument(
        's_buffer_end',
        default_value='2.0',
        description='Distance to fully return to raceline (m)'
    )
    
    trigger_distance_arg = DeclareLaunchArgument(
        'trigger_distance',
        default_value='1.5',
        description='Distance to opponent to start PREPARE_OVERTAKE (m)'
    )
    
    execute_distance_arg = DeclareLaunchArgument(
        'execute_distance',
        default_value='1.0',
        description='Distance to opponent to start OVERTAKE execution (m)'
    )
    
    overtake_timeout_arg = DeclareLaunchArgument(
        'overtake_timeout',
        default_value='5.0',
        description='Maximum time in OVERTAKE state (seconds)'
    )
    
    vehicle_width_arg = DeclareLaunchArgument(
        'vehicle_width',
        default_value='0.35',
        description='Vehicle width for clearance calculations (m)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for all published paths and markers'
    )
    
    # =========================================================================
    # Nodes
    # =========================================================================
    
    # Opponent Detector Node
    opponent_detector_node = Node(
        package='planning_pkg',
        executable='opponent_detector',
        name='opponent_detector',
        output='screen',
        parameters=[{
            'front_angle_min': LaunchConfiguration('front_angle_min'),
            'front_angle_max': LaunchConfiguration('front_angle_max'),
            'min_valid_range': 0.05,
            'max_detection_range': LaunchConfiguration('max_detection_range'),
            'lane_half_width': LaunchConfiguration('lane_half_width'),
            'min_ahead_margin': LaunchConfiguration('min_ahead_margin'),
            'max_ahead_distance': LaunchConfiguration('max_ahead_distance'),
            'enable_visualization': True,
            'frame_id': LaunchConfiguration('frame_id'),
            'detection_rate': 20.0,
        }]
    )
    
    # Global Overtake Planner Node
    global_overtake_planner_node = Node(
        package='planning_pkg',
        executable='global_overtake_planner',
        name='global_overtake_planner',
        output='screen',
        parameters=[{
            'd_max': LaunchConfiguration('d_max'),
            's_buffer_start': LaunchConfiguration('s_buffer_start'),
            's_overlap': LaunchConfiguration('s_overlap'),
            's_buffer_end': LaunchConfiguration('s_buffer_end'),
            'min_track_width': 2.0,
            'wall_safety_margin': 0.3,
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'min_opponent_distance': 0.5,
            'max_opponent_distance': 3.0,
            'trigger_distance': LaunchConfiguration('trigger_distance'),
            'execute_distance': LaunchConfiguration('execute_distance'),
            'overtake_timeout': LaunchConfiguration('overtake_timeout'),
            'return_duration': 2.0,
            'path_points': 50,
            'frame_id': LaunchConfiguration('frame_id'),
            'planner_rate': 20.0,
        }]
    )
    
    return LaunchDescription([
        # Opponent Detector arguments
        front_angle_min_arg,
        front_angle_max_arg,
        max_detection_range_arg,
        lane_half_width_arg,
        min_ahead_margin_arg,
        max_ahead_distance_arg,
        
        # Overtake Planner arguments
        d_max_arg,
        s_buffer_start_arg,
        s_overlap_arg,
        s_buffer_end_arg,
        trigger_distance_arg,
        execute_distance_arg,
        overtake_timeout_arg,
        vehicle_width_arg,
        frame_id_arg,
        
        # Nodes
        opponent_detector_node,
        global_overtake_planner_node,
    ])
