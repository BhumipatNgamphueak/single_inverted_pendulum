#!/usr/bin/env python3
"""
Launch file for Distributed Pendulum Control System
Starts all 4 nodes: Energy Swing-Up, LQR Stabilizer, Disturbance, and Manager
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package share directory
    pkg_share = FindPackageShare('single_inverted_pendulum')
    
    # Config file path
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'single_pendulum_swingup_params.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to parameter file'
    )
    
    # Node 1: Energy Swing-Up
    energy_node = Node(
        package='single_inverted_pendulum',
        executable='energy_swingup_node.py',
        name='energy_swingup_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    # Node 2: LQR Stabilizer
    lqr_node = Node(
        package='single_inverted_pendulum',
        executable='lqr_stabilizer_node.py',
        name='lqr_stabilizer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    # Node 3: Disturbance Generator
    disturbance_node = Node(
        package='single_inverted_pendulum',
        executable='disturbance_node.py',
        name='disturbance_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    # Node 4: Manager (Coordinator)
    manager_node = Node(
        package='single_inverted_pendulum',
        executable='manager_node.py',
        name='manager_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        energy_node,
        lqr_node,
        disturbance_node,
        manager_node,
    ])
