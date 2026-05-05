#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ur_moveit_config.launch_common import load_yaml


def generate_launch_description():
    # Configuration arguments
    ur_type = LaunchConfiguration("ur_type", default='ur3')
    onrobot_type = LaunchConfiguration("onrobot_type", default='rg2')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')
    planner_id = LaunchConfiguration('planner_id', default='RRTConnectkConfigDefault')
    ignore_if_busy = LaunchConfiguration('ignore_if_busy', default='true')
    trajectory_velocity_scaling = LaunchConfiguration('trajectory_velocity_scaling', default='0.1')
    trajectory_acceleration_scaling = LaunchConfiguration('trajectory_acceleration_scaling', default='0.1')
    connection_type = LaunchConfiguration('connection_type', default='serial')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')
    headless_mode = LaunchConfiguration('headless_mode', default='true')

    safety_limits = LaunchConfiguration("safety_limits", default='true')
    safety_pos_margin = LaunchConfiguration("safety_pos_margin", default='0.15')
    safety_k_position = LaunchConfiguration("safety_k_position", default='20')
    prefix = LaunchConfiguration("prefix", default='""')
    
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
    


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
        ),
        " ",
        "robot_ip:=",
        robot_ip,
        " ",
        "joint_limit_params:=",
        joint_limit_params,
        " ",
        "kinematics_params:=",
        kinematics_params,
        " ",
        "physical_params:=",
        physical_params,
        " ",
        "visual_params:=",
        visual_params,
        " ",
        "safety_limits:=",
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "name:=ur",
        " ",
        "ur_type:=",
        ur_type,
        " ",
        "onrobot_type:=",
        onrobot_type,
        " ",
        "script_filename:=ros_control.urscript",
        " ",
        "input_recipe_filename:=rtde_input_recipe.txt",
        " ",
        "output_recipe_filename:=rtde_output_recipe.txt",
        " ",
        "prefix:=",
        prefix,
        " ",
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
        ),
        " ",
        "name:==t",
        "ur",
        " ",
        "prefix:=",
        prefix,
        " ",
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "ur_moveit_config", "config/kinematics.yaml"
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ur_moveit_config", "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }

    ompl_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["ompl"].update(ompl_yaml)


    # 5. Run the MTC pick-and-place node
    mtc_node = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='ur3_mtc',
                executable='mtc_pick_place_kinematics',
                name='mtc_pick_place_kinematics',
                output='screen',
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                ]
            )
        ]
    )
    
    return LaunchDescription([
        # bringup,
        stop_program,
        start_program,
        activate_controller,
        mtc_node,
    ])