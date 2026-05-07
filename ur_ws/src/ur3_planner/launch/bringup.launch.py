from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration arguments
    name = LaunchConfiguration("name", default="ur_onrobot")
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')
    calibration_file = LaunchConfiguration('calibration_file', 
        default='~/git/RS2_SNL/ur_ws/src/ur3_planner/src/ur3_calibration.yaml')
    ur_type = LaunchConfiguration('ur_type', default='ur3')
    planner_id = LaunchConfiguration('planner_id', default='RRTConnectkConfigDefault')
    # launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    trajectory_velocity_scaling = LaunchConfiguration('trajectory_velocity_scaling', default='0.1')
    trajectory_acceleration_scaling = LaunchConfiguration('trajectory_acceleration_scaling', default='0.1')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
    use_fake_gripper = LaunchConfiguration('use_fake_gripper', default='true')
    gripper_connection_type = LaunchConfiguration('gripper_connection_type', default='tcp')
    headless_mode = LaunchConfiguration('headless_mode', default='false')
    ignore_if_busy = LaunchConfiguration('ignore_if_busy', default='true')
    initial_joint_controller = LaunchConfiguration("initial_joint_controller", default="scaled_joint_trajectory_controller")
    ip_address = LaunchConfiguration('ip_address', default='192.168.1.1')
    description_file = LaunchConfiguration('description_file', default='ur_onrobot.urdf.xacro')
    moveit_config_file = LaunchConfiguration('moveit_config_file', default='ur_onrobot.srdf.xacro')

    # 1. UR Driver
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            ])
        ]),
        launch_arguments={
            'name': name,
            'robot_ip': robot_ip,
            'calibration_file': calibration_file,
            'ur_type': ur_type,
            'initial_joint_controller': initial_joint_controller,
            'launch_rviz': 'false', # Default: false
            'trajectory_velocity_scaling': trajectory_velocity_scaling,
            'trajectory_acceleration_scaling': trajectory_acceleration_scaling,
            'use_fake_gripper': use_fake_gripper,
            'gripper_connection_type': gripper_connection_type,
            'headless_mode': headless_mode,
            'description_file': description_file,
            'moveit_config_file': moveit_config_file,
        }.items()
    )

    # 2. MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'name': name,
            'ur_type': ur_type,
            'launch_rviz': rviz,
            'description_file': description_file, # Use the URDF with the gripper attached
        }.items()
    )

    # 3. Listener node (RRTConnectkConfigDefault/RRTstarkConfigDefault)
    listener_node = Node(
        package='ur3_planner',
        executable='ur3_planner_listener',
        name='ur3_planner_listener',
        parameters=[{
            'planning_group': 'ur_manipulator',
            'execute_immediately': True,
            'ignore_if_busy': ignore_if_busy,
            'planner_id': planner_id,
        }],
        output='screen'
    )

    # 4. Spawn gripper controllers
    spawn_gripper_traj_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['finger_width_trajectory_controller'],
        output='screen'
    )
    # 5. Spawn gripper controller
    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['finger_width_controller'],
        output='screen'
    )

    # 6. Activate ur scaled joint trajectory controller
    activate_ur_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate', 'scaled_joint_trajectory_controller'],
        output='screen'
    )
    # 7. Activate gripper trajectory controller
    activate_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate', 'finger_width_trajectory_controller'],
        output='screen'
    )

    # 8. Goal Pose Publisher GUI
    gui = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution([
            FindPackageShare('ur3_planner'), 'src', 'ur3_goal_gui.py'
        ])],
        output='screen'
    )
    
    

    return LaunchDescription([
        ur_driver,
        moveit,
        listener_node,
        spawn_gripper_traj_controller,
        spawn_gripper_controller,
        activate_ur_controller,
        activate_gripper_controller,
        gui,
    ])

