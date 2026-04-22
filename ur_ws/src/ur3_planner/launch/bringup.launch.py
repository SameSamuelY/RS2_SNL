from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration arguments
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')
    calibration_file = LaunchConfiguration('calibration_file', 
        default='~/git/RS2_SNL/ur_ws/src/ur3_planner/src/ur3_calibration.yaml')
    ur_type = LaunchConfiguration('ur_type', default='ur3')
    planner_id = LaunchConfiguration('planner_id', default='RRTConnectkConfigDefault')
    # launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    trajectory_velocity_scaling = LaunchConfiguration('trajectory_velocity_scaling', default='0.3')
    trajectory_acceleration_scaling = LaunchConfiguration('trajectory_acceleration_scaling', default='0.3')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
    use_fake_gripper = LaunchConfiguration('use_fake_gripper', default='true')
    gripper_connection_type = LaunchConfiguration('gripper_connection_type', default='tcp')
    headless_mode = LaunchConfiguration('headless_mode', default='false')
    ignore_if_busy = LaunchConfiguration('ignore_if_busy', default='false')

    # 1. UR Driver
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'calibration_file': calibration_file,
            'ur_type': ur_type,
            'launch_rviz': 'false', # Default: false
            'trajectory_velocity_scaling': trajectory_velocity_scaling,
            'trajectory_acceleration_scaling': trajectory_acceleration_scaling,
            'use_fake_gripper': use_fake_gripper,
            'gripper_connection_type': gripper_connection_type,
            'headless_mode': headless_mode,
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
            'ur_type': ur_type,
            'launch_rviz': 'true',
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

    # Activate controller
    activate_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate', 'scaled_joint_trajectory_controller'],
        output='screen'
    )

    # 4. Gripper driver
    gripper_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'launch',
                'onrobot_control.launch.py'
            ])
        ]),
        launch_arguments={
            'onrobot_type': 'rg2',
            'connection_type': gripper_connection_type,
            'use_fake_hardware': use_fake_gripper,
            'ip_address': '192.168.1.100',   # ignored when fake
        }.items()
    )

    # 5. Goal Pose Publisher GUI
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
        # gripper_driver,
    ])

