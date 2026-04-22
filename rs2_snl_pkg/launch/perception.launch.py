from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rs2_snl_pkg',
            executable='shape_colour_detector_node',
            name='shape_colour_detector_node',
            output='screen'
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[
                '/home/lachlanselleck/ros2_ws/src/RS2_SNL/rs2_snl_pkg/config/apriltag.yaml'
            ],
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info')
            ]
        ),
    ])
