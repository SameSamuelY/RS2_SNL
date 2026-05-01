#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration arguments
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')
    planner_id = LaunchConfiguration('planner_id', default='RRTConnectkConfigDefault')
    ignore_if_busy = LaunchConfiguration('ignore_if_busy', default='true')
    trajectory_velocity_scaling = LaunchConfiguration('trajectory_velocity_scaling', default='0.1')
    trajectory_acceleration_scaling = LaunchConfiguration('trajectory_acceleration_scaling', default='0.1')
    connection_type = LaunchConfiguration('connection_type', default='serial')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')
    headless_mode = LaunchConfiguration('headless_mode', default='true')
    
    # 1. Include the main bringup launch
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
            'use_fake_hardware': 'false', # Force real hardware for MTC testing
            'headless_mode': 'false', # Force RViz for MTC testing
        }.items()
    )
    
    # 2. Start the External Control program via dashboard
    # Stop the program (if running) and then start it
    stop_program = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/dashboard_client/stop', 'std_srvs/srv/Trigger', '{}'],
                output='screen'
            )
        ]
    )
    start_program = TimerAction(
        period=6.0,  # wait 1 second after stop
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/dashboard_client/play', 'std_srvs/srv/Trigger', '{}'],
                output='screen'
            )
        ]
    )
    
    # 3. Activate trajectory controller
    activate_controller = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'switch_controllers', '--activate', 'scaled_joint_trajectory_controller'],
                output='screen'
            )
        ]
    )
    
    # 4. Set kinematics parameter (if needed for Cartesian planning)
    set_kinematics = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', PathJoinSubstitution([FindPackageShare('ur3_mtc'), 'scripts', 'set_kinematics.py'])],
                output='screen'
            )
        ]
    )
    
    # 5. Run the MTC pick-and-place node
    mtc_node = TimerAction(
        period=16.0,
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
        stop_program,
        start_program,
        activate_controller,
        # set_kinematics,
        mtc_node,
    ])