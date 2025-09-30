#!/usr/bin/env python3
"""
LunaBot Navigation Stack Launch File
Launches Nav2 navigation stack with SLAM and localization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    # Package directories
    pkg_lunabot_navigation = FindPackageShare('lunabot_navigation')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_lunabot_navigation, 'maps', 'lunar_habitat.yaml']),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_lunabot_navigation, 'config', 'nav2_params.yaml']),
        description='Full path to param file to load'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composed bringup'
    )
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='Whether to respawn if a node crashes'
    )
    
    # Variables
    lifecycle_nodes = ['controller_server',
                      'smoother_server',
                      'planner_server',
                      'behavior_server',
                      'bt_navigator',
                      'waypoint_follower',
                      'velocity_smoother']
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # SLAM Toolbox
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_lunabot_navigation, 'config', 'mapper_params_online_async.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        remappings=remappings
    )
    
    # Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn
        }.items()
    )
    
    # Localization (when not using SLAM)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'localization_launch.py'])
        ),
        condition=IfCondition(['not ', slam]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn
        }.items()
    )
    
    # Lifecycle manager for SLAM
    lifecycle_manager_slam = Node(
        condition=IfCondition(slam),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['slam_toolbox']}]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    
    # Add nodes and launch files
    ld.add_action(slam_toolbox_node)
    ld.add_action(lifecycle_manager_slam)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(localization_launch)
    
    return ld