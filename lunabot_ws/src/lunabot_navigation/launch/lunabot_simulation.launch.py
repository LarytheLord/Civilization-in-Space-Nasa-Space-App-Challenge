#!/usr/bin/env python3
"""
LunaBot Webots Simulation Launch File
Launches Webots simulation with lunar habitat world and LunaBot robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def launch_webots(context, *args, **kwargs):
    """
    OpaqueFunction to launch Webots.
    This is necessary to resolve the LaunchConfiguration for the world file path.
    """
    world_file_path = LaunchConfiguration('world_file').perform(context)

    webots = WebotsLauncher(
        world=world_file_path,
        ros2_supervisor=True
    )
    return [webots]

def generate_launch_description():
    
    # Package directories
    pkg_lunabot_navigation = get_package_share_directory('lunabot_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_lunabot_navigation, 'worlds', 'lunar_habitat.wbt'),
        description='Full path to Webots world file to load'
    )
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='lunabot',
        description='Name of the robot'
    )
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'
    )
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', 
        default_value='0.0',
        description='Initial y position of the robot'
    )
    
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial z position of the robot'
    )
    
    # Webots launcher
    webots_launcher_function = OpaqueFunction(function=launch_webots)
    
    # Robot controller
    lunabot_driver_node = Node(
        package='lunabot_navigation',
        executable='lunabot_controller.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Robot state publisher (for TF tree)
    urdf_file = os.path.join(pkg_lunabot_navigation, 'urdf', 'lunabot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_lunabot_navigation, 'config', 'lunabot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    
    # Add nodes
    ld.add_action(webots_launcher_function)
    ld.add_action(lunabot_driver_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld