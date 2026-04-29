#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration arguments (pass through to bringup)
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')
    planner_id = LaunchConfiguration('planner_id', default='RRTConnectkConfigDefault')
    ignore_if_busy = LaunchConfiguration('ignore_if_busy', default='true')
    trajectory_velocity_scaling = LaunchConfiguration('trajectory_velocity_scaling', default='0.1')
    trajectory_acceleration_scaling = LaunchConfiguration('trajectory_acceleration_scaling', default='0.1')
    connection_type = LaunchConfiguration('connection_type', default='serial')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')
    headless_mode = LaunchConfiguration('headless_mode', default='true')
    
    # 1. Include the main bringup launch (UR driver + MoveIt + RViz)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ur3_planner'), 'launch', 'bringup.launch.py'])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'planner_id': planner_id,
            'ignore_if_busy': ignore_if_busy,
            'trajectory_velocity_scaling': trajectory_velocity_scaling,
            'trajectory_acceleration_scaling': trajectory_acceleration_scaling,
            'connection_type': connection_type,
            'use_fake_hardware': use_fake_hardware,
            'headless_mode': headless_mode,
        }.items()
    )
    
    # 2. Set kinematics parameter using a one‑time Python script
    kinematics_script = PathJoinSubstitution([
        FindPackageShare('ur3_mtc'), 'scripts', 'set_kinematics.py'
    ])
    set_kinematics = TimerAction(
        period=6.0,  # after controller activation
        actions=[
            ExecuteProcess(
                cmd=['python3', kinematics_script],
                output='screen'
            )
        ]
    )
    
    # 3. Run the MTC pick‑and‑place node
    mtc_node = TimerAction(
        period=7.0,  # after kinematics is set
        actions=[
            Node(
                package='ur3_mtc',
                executable='mtc_pick_place',
                name='mtc_pick_place',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # bringup,
        set_kinematics,
        mtc_node,
    ])