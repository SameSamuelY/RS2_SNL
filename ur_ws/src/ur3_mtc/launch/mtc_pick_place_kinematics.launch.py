#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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

    description_package = LaunchConfiguration("ur_description_package", default='ur_description')
    description_file = LaunchConfiguration("description_file", default='ur_onrobot.urdf.xacro')
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package", default='ur_moveit_config')
    moveit_config_file = LaunchConfiguration("moveit_config_file", default='ur_onrobot.srdf.xacro')
    
    # === Pick and place positions ===
    pick_x = LaunchConfiguration('pick_x', default='0.3')
    pick_y = LaunchConfiguration('pick_y', default='-0.2')
    pick_z = LaunchConfiguration('pick_z', default='0.03')
    place_x = LaunchConfiguration('place_x', default='-0.3')
    place_y = LaunchConfiguration('place_y', default='0.2')
    place_z = LaunchConfiguration('place_z', default='-0.5')
    place_qx = LaunchConfiguration('place_qx', default='0.0')
    place_qy = LaunchConfiguration('place_qy', default='0.0')
    place_qz = LaunchConfiguration('place_qz', default='0.0')
    place_qw = LaunchConfiguration('place_qw', default='1.0')


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
            'description_file': description_file,
            'moveit_config_file': moveit_config_file,
        }.items()
    )
    
    # 2. Start the External Control program via dashboard
    # Stop the program (if running) and then start it
    stop_program = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/dashboard_client/stop', 'std_srvs/srv/Trigger', '{}'],
                output='screen'
            )
        ]
    )
    start_program = TimerAction(
        period=2.0,  # wait 1 second after stop
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/dashboard_client/play', 'std_srvs/srv/Trigger', '{}'],
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
            [FindPackageShare(description_package), "urdf", description_file]
        ),
        " ",
        "robot_ip:=xxx.yyy.zzz.www",
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
        "name:=ur_onrobot",
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
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
        ),
        " ",
        "name:=",
        "ur_onrobot",
        " ",
        "prefix:=",
        prefix,
        " ",
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
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


    # 3. Run the MTC pick-and-place node
    mtc_node = TimerAction(
        period=3.0,
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
                    {'pick_x': pick_x},
                    {'pick_y': pick_y},
                    {'pick_z': pick_z},
                    {'place_x': place_x},
                    {'place_y': place_y},
                    {'place_z': place_z},
                    {'place_qx': place_qx},
                    {'place_qy': place_qy},
                    {'place_qz': place_qz},
                    {'place_qw': place_qw},
                ]
            )
        ]
    )

    # 4. Activate ur scaled joint trajectory controller
    activate_ur_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate', 'scaled_joint_trajectory_controller'],
        output='screen'
    )
    # 5. Activate gripper trajectory controller
    activate_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate', 'finger_width_trajectory_controller'],
        output='screen'
    )

    
    return LaunchDescription([
        # bringup,
        stop_program,
        start_program,
        mtc_node,
        activate_ur_controller,
        activate_gripper_controller,
    ])