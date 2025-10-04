#!/usr/bin/env python3
"""
LunaBot Complete System Launch File
Launches the entire LunaBot system with all components
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Package directories
    pkg_lunabot_navigation = FindPackageShare('lunabot_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    enable_ai = LaunchConfiguration('enable_ai')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([pkg_lunabot_navigation, 'worlds', 'lunar_habitat.world']),
        description='Full path to world file to load'
    )
    
    declare_enable_ai_cmd = DeclareLaunchArgument(
        'enable_ai',
        default_value='true',
        description='Enable AI decision engine'
    )
    
    declare_enable_rviz_cmd = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    # 1. Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_lunabot_navigation, 'launch', 'lunabot_simulation.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file
        }.items()
    )
    
    # 2. Launch Navigation Stack (delayed to allow Gazebo to start)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_lunabot_navigation, 'launch', 'navigation.launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )
    
    # 3. Sensor Fusion Node
    sensor_fusion_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='sensor_fusion_node.py',
                name='sensor_fusion_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 4. Navigation Controller
    navigation_controller_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='navigation_controller.py',
                name='navigation_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 5. Habitat Monitor
    habitat_monitor_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='habitat_monitor.py',
                name='habitat_monitor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 6. Maintenance Patrol
    maintenance_patrol_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='maintenance_patrol.py',
                name='maintenance_patrol',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # 7. AI Decision Engine (conditional)
    ai_decision_engine_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='ai_decision_engine.py',
                name='ai_decision_engine',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(enable_ai)
            )
        ]
    )
    
    # 8. Demo Controller for automated demonstration
    demo_controller_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='demo_controller.py',
                name='demo_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 9. Habitat Site Analyzer
    habitat_analyzer_node = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='habitat_site_analyzer.py',
                name='habitat_site_analyzer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 10. WebSocket Bridge
    websocket_bridge_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='lunabot_navigation',
                executable='websocket_bridge.py',
                name='websocket_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_enable_ai_cmd)
    ld.add_action(declare_enable_rviz_cmd)
    
    # Add launch sequences
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)
    ld.add_action(sensor_fusion_node)
    ld.add_action(navigation_controller_node)
    ld.add_action(habitat_monitor_node)
    ld.add_action(maintenance_patrol_node)
    ld.add_action(ai_decision_engine_node)
    ld.add_action(demo_controller_node)
    ld.add_action(habitat_analyzer_node)
    ld.add_action(websocket_bridge_node)
    
    return ld
