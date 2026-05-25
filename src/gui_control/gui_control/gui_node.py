#!/usr/bin/env python3

import sys
import json
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool, Float64, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QTextEdit,
    QFrame,
    QSizePolicy,
    QDoubleSpinBox,
    QMainWindow,
    QSlider,
    QScrollArea,
    QComboBox,
)


class GuiROSNode(Node):
    def __init__(self):
        super().__init__('gui_controller')

        self.goal_publisher = self.create_publisher(Pose, '/ur3_goal_pose', 10)
        self.plan_goal_publisher = self.create_publisher(PoseStamped, '/plan_goal_pose', 10)
        self.trigger_publisher = self.create_publisher(Bool, '/trigger_pick_and_place', 10)
        self.command_publisher = self.create_publisher(String, '/gui_command', 10)

        self.gripper_publisher = self.create_publisher(
            Float64MultiArray,
            '/finger_width_controller/commands',
            10
        )

        self.velocity_publisher = self.create_publisher(Float64, '/velocity_scale', 10)

        self.camera_topic = '/camera/camera/color/image_raw'
        # For WSL burger test, change to:
        # self.camera_topic = '/image'

        self.bridge = CvBridge()
        self.latest_camera_frame = None
        self.latest_camera_error = ""

        self.latest_motion_status = "idle"
        self.latest_detection_result = []
        self.latest_detection_error = ""

        self.camera_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )

        self.motion_status_subscriber = self.create_subscription(
            String,
            '/motion_status',
            self.motion_status_callback,
            10
        )

        self.detection_subscriber = self.create_subscription(
            String,
            '/detection_result',
            self.detection_callback,
            10
        )

        self.get_logger().info("GUI ROS node ready")

    def publish_goal(self, pose: Pose):
        self.goal_publisher.publish(pose)
        self.get_logger().info("Published pick goal pose to /ur3_goal_pose")

    def publish_place_goal(self, pose: Pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose = pose
        self.plan_goal_publisher.publish(msg)
        self.get_logger().info("Published place goal pose to /plan_goal_pose")

    def publish_pick_and_place_trigger(self):
        msg = Bool()
        msg.data = True
        self.trigger_publisher.publish(msg)
        self.get_logger().info("Published trigger to /trigger_pick_and_place")

    def publish_command(self, command_text: str):
        msg = String()
        msg.data = command_text
        self.command_publisher.publish(msg)

    def publish_gripper_width(self, width: float):
        msg = Float64MultiArray()
        msg.data = [width]
        self.gripper_publisher.publish(msg)
        self.get_logger().info(f"Published gripper width: {width:.3f} m")

    def publish_velocity_scale(self, scale: float):
        msg = Float64()
        msg.data = scale
        self.velocity_publisher.publish(msg)

    def camera_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_camera_frame = frame
            self.latest_camera_error = ""
        except Exception as error:
            self.latest_camera_error = str(error)

    def motion_status_callback(self, msg: String):
        self.latest_motion_status = msg.data

    def detection_callback(self, msg: String):
        try:
            parsed_data = json.loads(msg.data)

            # Supported /detection_result formats:
            # 1. [{"tag_x_m": ..., "tag_y_m": ..., "tag_z_m": ...}]
            # 2. {"objects": [{"position": {"x": ..., "y": ..., "z": ...}}]}
            # 3. {"position": {"x": ..., "y": ..., "z": ...}}
            if isinstance(parsed_data, list):
                detections = parsed_data
            elif isinstance(parsed_data, dict) and isinstance(parsed_data.get("objects"), list):
                detections = parsed_data["objects"]
            elif isinstance(parsed_data, dict):
                detections = [parsed_data]
            else:
                detections = []

            self.latest_detection_result = detections
            self.latest_detection_error = ""

        except Exception as error:
            self.latest_detection_result = []
            self.latest_detection_error = str(error)


class RobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        rclpy.init(args=sys.argv)
        self.ros_node = GuiROSNode()

        self.current_state = "Idle"
        self.last_processed_motion_status = ""

        self.init_ui()
        self.init_ros_timers()

    def init_ui(self):
        self.setWindowTitle("Subsystem 3 - Interaction and Execution")
        self.setGeometry(100, 80, 1200, 760)
        self.setMinimumSize(1200, 720)

        self.setStyleSheet("""
            QWidget {
                background-color: #f4f6f8;
                font-family: Arial;
                font-size: 12px;
            }
            QGroupBox {
                background-color: white;
                border: 1px solid #d9dee3;
                border-radius: 10px;
                margin-top: 12px;
                font-weight: bold;
                padding-top: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px 0 4px;
                color: #1f2d3d;
            }
            QLabel#StateLabel {
                font-size: 16px;
                font-weight: bold;
                color: #1f2d3d;
            }
            QLabel#StateValue {
                font-size: 16px;
                font-weight: bold;
                padding: 8px 14px;
                border-radius: 8px;
                background-color: #e9ecef;
                color: #212529;
            }
            QLabel#InfoLabel {
                font-size: 12px;
                color: #344054;
                padding: 2px 0;
            }
            QPushButton {
                border: none;
                border-radius: 8px;
                padding: 8px;
                font-size: 12px;
                font-weight: bold;
                min-height: 32px;
                background-color: #344054;
                color: white;
            }
            QPushButton:hover {
                background-color: #1f2937;
            }
            QPushButton#StartButton {
                background-color: #2e7d32;
            }
            QPushButton#StopButton {
                background-color: #c62828;
            }
            QPushButton#ResetButton {
                background-color: #1565c0;
            }
            QTextEdit {
                background-color: #fcfcfd;
                border: 1px solid #d0d5dd;
                border-radius: 8px;
                padding: 8px;
                color: #1f2937;
            }
            QFrame#CameraFrame {
                background-color: #e8edf2;
                border: 2px dashed #98a2b3;
                border-radius: 10px;
            }
            QLabel#CameraPlaceholder {
                color: #667085;
                font-size: 14px;
            }
            QDoubleSpinBox, QComboBox {
                background-color: #ffffff;
                border: 1px solid #d0d5dd;
                border-radius: 6px;
                padding: 4px;
                min-height: 24px;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(10)
        main_layout.addLayout(self.build_content())

        central_widget.setLayout(main_layout)

    def init_ros_timers(self):
        self.ros_spin_timer = QTimer()
        self.ros_spin_timer.timeout.connect(self.spin_ros_once)
        self.ros_spin_timer.start(20)

        self.status_refresh_timer = QTimer()
        self.status_refresh_timer.timeout.connect(self.refresh_motion_status)
        self.status_refresh_timer.start(100)

        self.camera_refresh_timer = QTimer()
        self.camera_refresh_timer.timeout.connect(self.refresh_camera_feed)
        self.camera_refresh_timer.start(33)

        self.detection_refresh_timer = QTimer()
        self.detection_refresh_timer.timeout.connect(self.refresh_detection_display)
        self.detection_refresh_timer.start(250)

    def spin_ros_once(self):
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def build_content(self):
        content_layout = QHBoxLayout()
        content_layout.setSpacing(12)

        left_content = QWidget()
        left_panel = QVBoxLayout(left_content)
        left_panel.setSpacing(8)
        left_panel.setContentsMargins(4, 4, 4, 4)

        left_panel.addWidget(self.build_control_group())
        left_panel.addWidget(self.build_goal_group())
        left_panel.addWidget(self.build_place_goal_group())
        left_panel.addWidget(self.build_gripper_group())
        left_panel.addWidget(self.build_velocity_group())
        left_panel.addWidget(self.build_workspace_limits_group())
        left_panel.addWidget(self.build_status_group())
        left_panel.addWidget(self.build_motion_group())
        left_panel.addWidget(self.build_log_group())
        left_panel.addStretch()

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_content)
        left_scroll.setMinimumWidth(700)

        right_panel = QVBoxLayout()
        right_panel.setSpacing(8)

        right_panel.addWidget(self.build_camera_group(), 2)
        right_panel.addWidget(self.build_detection_group(), 1)
        right_panel.addWidget(self.build_info_group(), 0)

        content_layout.addWidget(left_scroll, 1)
        content_layout.addLayout(right_panel, 1)

        return content_layout

    def make_spinbox(self, minimum, maximum, value, step=0.01):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setDecimals(3)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def build_control_group(self):
        group = QGroupBox("System Controls")
        layout = QVBoxLayout()

        instruction = QLabel("Start publishes /trigger_pick_and_place and sends the place pose to /plan_goal_pose.")
        instruction.setWordWrap(True)
        instruction.setObjectName("InfoLabel")

        self.start_button = QPushButton("Start Pick-and-Place")
        self.start_button.setObjectName("StartButton")

        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("StopButton")

        self.reset_button = QPushButton("Reset")
        self.reset_button.setObjectName("ResetButton")

        self.start_button.clicked.connect(self.start_system_with_goal)
        self.stop_button.clicked.connect(self.stop_system)
        self.reset_button.clicked.connect(self.reset_system)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.reset_button)

        layout.addWidget(instruction)
        layout.addLayout(button_layout)

        group.setLayout(layout)
        return group

    def build_goal_group(self):
        group = QGroupBox("Pick / Cartesian Goal Pose")
        layout = QGridLayout()

        self.goal_x_input = self.make_spinbox(-1.0, 1.0, -0.300)
        self.goal_y_input = self.make_spinbox(-1.0, 1.0, -0.200)
        self.goal_z_input = self.make_spinbox(0.0, 1.5, 0.010)

        self.qx_input = self.make_spinbox(-1.0, 1.0, 0.000, 0.001)
        self.qy_input = self.make_spinbox(-1.0, 1.0, 0.707, 0.001)
        self.qz_input = self.make_spinbox(-1.0, 1.0, 0.000, 0.001)
        self.qw_input = self.make_spinbox(-1.0, 1.0, 0.707, 0.001)

        self.send_pose_button = QPushButton("Send Pick Pose")
        self.send_pose_button.clicked.connect(self.send_pick_goal_only)

        layout.addWidget(QLabel("X:"), 0, 0)
        layout.addWidget(self.goal_x_input, 0, 1)
        layout.addWidget(QLabel("Y:"), 0, 2)
        layout.addWidget(self.goal_y_input, 0, 3)
        layout.addWidget(QLabel("Z:"), 0, 4)
        layout.addWidget(self.goal_z_input, 0, 5)

        layout.addWidget(QLabel("qx:"), 1, 0)
        layout.addWidget(self.qx_input, 1, 1)
        layout.addWidget(QLabel("qy:"), 1, 2)
        layout.addWidget(self.qy_input, 1, 3)
        layout.addWidget(QLabel("qz:"), 1, 4)
        layout.addWidget(self.qz_input, 1, 5)

        layout.addWidget(QLabel("qw:"), 2, 0)
        layout.addWidget(self.qw_input, 2, 1)
        layout.addWidget(self.send_pose_button, 3, 0, 1, 6)

        group.setLayout(layout)
        return group

    def build_place_goal_group(self):
        group = QGroupBox("Place Pose for /plan_goal_pose")
        layout = QGridLayout()

        self.place_x_input = self.make_spinbox(-1.0, 1.0, 0.300)
        self.place_y_input = self.make_spinbox(-1.0, 1.0, 0.200)
        self.place_z_input = self.make_spinbox(0.0, 1.5, 0.150)

        self.place_qx_input = self.make_spinbox(-1.0, 1.0, 0.000, 0.001)
        self.place_qy_input = self.make_spinbox(-1.0, 1.0, 0.000, 0.001)
        self.place_qz_input = self.make_spinbox(-1.0, 1.0, 0.000, 0.001)
        self.place_qw_input = self.make_spinbox(-1.0, 1.0, 1.000, 0.001)

        self.send_place_button = QPushButton("Send Place Pose")
        self.send_place_button.clicked.connect(self.send_place_goal_only)

        layout.addWidget(QLabel("X:"), 0, 0)
        layout.addWidget(self.place_x_input, 0, 1)
        layout.addWidget(QLabel("Y:"), 0, 2)
        layout.addWidget(self.place_y_input, 0, 3)
        layout.addWidget(QLabel("Z:"), 0, 4)
        layout.addWidget(self.place_z_input, 0, 5)

        layout.addWidget(QLabel("qx:"), 1, 0)
        layout.addWidget(self.place_qx_input, 1, 1)
        layout.addWidget(QLabel("qy:"), 1, 2)
        layout.addWidget(self.place_qy_input, 1, 3)
        layout.addWidget(QLabel("qz:"), 1, 4)
        layout.addWidget(self.place_qz_input, 1, 5)

        layout.addWidget(QLabel("qw:"), 2, 0)
        layout.addWidget(self.place_qw_input, 2, 1)
        layout.addWidget(self.send_place_button, 3, 0, 1, 6)

        group.setLayout(layout)
        return group

    def build_gripper_group(self):
        group = QGroupBox("Gripper Control")
        layout = QVBoxLayout()

        self.gripper_width_label = QLabel("Width: 0.050 m")
        self.gripper_width_label.setObjectName("InfoLabel")

        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setMinimum(0)
        self.gripper_slider.setMaximum(110)
        self.gripper_slider.setValue(50)
        self.gripper_slider.valueChanged.connect(self.update_gripper_label)

        self.send_gripper_button = QPushButton("Send Gripper Command")
        self.send_gripper_button.clicked.connect(self.send_gripper_command)

        note_label = QLabel("Publishes Float64MultiArray to /finger_width_controller/commands")
        note_label.setObjectName("InfoLabel")
        note_label.setWordWrap(True)

        layout.addWidget(self.gripper_width_label)
        layout.addWidget(self.gripper_slider)
        layout.addWidget(note_label)
        layout.addWidget(self.send_gripper_button)

        group.setLayout(layout)
        return group

    def build_velocity_group(self):
        group = QGroupBox("Velocity Control")
        layout = QVBoxLayout()

        self.velocity_label = QLabel("Velocity Scale: 0.50")
        self.velocity_label.setObjectName("InfoLabel")

        self.velocity_slider = QSlider(Qt.Horizontal)
        self.velocity_slider.setMinimum(10)
        self.velocity_slider.setMaximum(100)
        self.velocity_slider.setValue(50)
        self.velocity_slider.valueChanged.connect(self.update_velocity_label)

        self.send_velocity_button = QPushButton("Send Velocity Scale")
        self.send_velocity_button.clicked.connect(self.send_velocity_scale)

        note_label = QLabel("Range: 0.10 to 1.00")
        note_label.setObjectName("InfoLabel")

        layout.addWidget(self.velocity_label)
        layout.addWidget(self.velocity_slider)
        layout.addWidget(note_label)
        layout.addWidget(self.send_velocity_button)

        group.setLayout(layout)
        return group

    def build_workspace_limits_group(self):
        group = QGroupBox("Safe Workspace Limits")
        layout = QGridLayout()

        self.x_min_input = self.make_spinbox(-1.0, 1.0, -0.600)
        self.x_max_input = self.make_spinbox(-1.0, 1.0, 0.600)

        self.y_min_input = self.make_spinbox(-1.0, 1.0, -0.600)
        self.y_max_input = self.make_spinbox(-1.0, 1.0, 0.600)

        self.z_min_input = self.make_spinbox(0.0, 1.5, 0.000)
        self.z_max_input = self.make_spinbox(0.0, 1.5, 0.500)

        self.workspace_status_label = QLabel("Workspace validation active.")
        self.workspace_status_label.setObjectName("InfoLabel")
        self.workspace_status_label.setWordWrap(True)

        layout.addWidget(QLabel("X min:"), 0, 0)
        layout.addWidget(self.x_min_input, 0, 1)
        layout.addWidget(QLabel("X max:"), 0, 2)
        layout.addWidget(self.x_max_input, 0, 3)

        layout.addWidget(QLabel("Y min:"), 1, 0)
        layout.addWidget(self.y_min_input, 1, 1)
        layout.addWidget(QLabel("Y max:"), 1, 2)
        layout.addWidget(self.y_max_input, 1, 3)

        layout.addWidget(QLabel("Z min:"), 2, 0)
        layout.addWidget(self.z_min_input, 2, 1)
        layout.addWidget(QLabel("Z max:"), 2, 2)
        layout.addWidget(self.z_max_input, 2, 3)

        layout.addWidget(self.workspace_status_label, 3, 0, 1, 4)

        group.setLayout(layout)
        return group

    def build_status_group(self):
        group = QGroupBox("System Status")
        layout = QGridLayout()

        state_label = QLabel("Current State:")
        state_label.setObjectName("StateLabel")

        self.state_value = QLabel("Idle")
        self.state_value.setObjectName("StateValue")
        self.state_value.setAlignment(Qt.AlignCenter)

        self.status_message = QLabel("System is waiting for user input.")
        self.status_message.setObjectName("InfoLabel")
        self.status_message.setWordWrap(True)

        layout.addWidget(state_label, 0, 0)
        layout.addWidget(self.state_value, 0, 1)
        layout.addWidget(self.status_message, 1, 0, 1, 2)

        group.setLayout(layout)
        return group

    def build_motion_group(self):
        group = QGroupBox("Motion Planning Feedback")
        layout = QVBoxLayout()

        self.motion_status_label = QLabel("Motion Status: idle")
        self.motion_status_label.setObjectName("InfoLabel")

        self.motion_stage_label = QLabel("Stage: Waiting for command")
        self.motion_stage_label.setObjectName("InfoLabel")

        layout.addWidget(self.motion_status_label)
        layout.addWidget(self.motion_stage_label)

        group.setLayout(layout)
        return group

    def build_log_group(self):
        group = QGroupBox("Command Log")
        layout = QVBoxLayout()

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(120)

        layout.addWidget(self.log_box)
        group.setLayout(layout)

        self.append_log("GUI initialised successfully.")
        self.append_log("Subscribed to camera, detection and motion status topics.")
        self.append_log("Place pose publishes to /plan_goal_pose.")
        self.append_log("Start button publishes to /trigger_pick_and_place.")

        return group

    def build_camera_group(self):
        group = QGroupBox("Live Camera Feed")
        layout = QVBoxLayout()

        camera_frame = QFrame()
        camera_frame.setObjectName("CameraFrame")
        camera_frame.setMinimumHeight(300)
        camera_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        frame_layout = QVBoxLayout()
        frame_layout.setAlignment(Qt.AlignCenter)

        self.camera_label = QLabel(
            "Waiting for camera feed...\n\n"
            "RealSense: /camera/camera/color/image_raw\n"
            "WSL test: /image"
        )
        self.camera_label.setObjectName("CameraPlaceholder")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(480, 300)

        self.camera_topic_label = QLabel(f"Topic: {self.ros_node.camera_topic}")
        self.camera_topic_label.setObjectName("InfoLabel")
        self.camera_topic_label.setAlignment(Qt.AlignCenter)

        frame_layout.addWidget(self.camera_label)
        camera_frame.setLayout(frame_layout)

        layout.addWidget(camera_frame)
        layout.addWidget(self.camera_topic_label)

        group.setLayout(layout)
        return group

    def build_detection_group(self):
        group = QGroupBox("Perception Detection Result")
        layout = QVBoxLayout()

        self.detection_count_label = QLabel("Detected Objects: 0")
        self.detection_count_label.setObjectName("InfoLabel")

        self.detection_selector = QComboBox()
        self.detection_selector.currentIndexChanged.connect(self.update_selected_detection_display)

        self.selected_detection_label = QLabel("Selected Detection:\nNo object detected yet.")
        self.selected_detection_label.setObjectName("InfoLabel")
        self.selected_detection_label.setWordWrap(True)

        self.use_detection_button = QPushButton("Use Selected Detection as Pick Pose")
        self.use_detection_button.clicked.connect(self.use_selected_detection_as_pick_pose)

        self.detection_note_label = QLabel("Supports object position format and tag_x_m/tag_y_m/tag_z_m format.")
        self.detection_note_label.setObjectName("InfoLabel")
        self.detection_note_label.setWordWrap(True)

        layout.addWidget(self.detection_count_label)
        layout.addWidget(self.detection_selector)
        layout.addWidget(self.selected_detection_label)
        layout.addWidget(self.use_detection_button)
        layout.addWidget(self.detection_note_label)

        group.setLayout(layout)
        return group

    def build_info_group(self):
        group = QGroupBox("Subsystem Information")
        layout = QVBoxLayout()

        self.mode_label = QLabel("Mode: Manual GUI Control + Perception Monitoring")
        self.connection_label = QLabel("ROS2 Connection: Active")
        self.last_command_label = QLabel("Last Command: None")

        self.integration_label = QLabel(
            "Topics: /detection_result, /ur3_goal_pose, /plan_goal_pose, "
            "/trigger_pick_and_place, /finger_width_controller/commands, "
            "/velocity_scale, /gui_command, /motion_status, camera image topic"
        )
        self.integration_label.setWordWrap(True)

        for label in [
            self.mode_label,
            self.connection_label,
            self.last_command_label,
            self.integration_label,
        ]:
            label.setObjectName("InfoLabel")
            layout.addWidget(label)

        group.setLayout(layout)
        return group

    def append_log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_box.append(f"[{timestamp}] {message}")

    def update_state_display(self, state, message):
        self.current_state = state
        self.state_value.setText(state)
        self.status_message.setText(message)

    def normalize_pose_quaternion(self, pose):
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        norm = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5

        if norm == 0:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.707
            pose.orientation.z = 0.0
            pose.orientation.w = 0.707
            return pose

        pose.orientation.x = qx / norm
        pose.orientation.y = qy / norm
        pose.orientation.z = qz / norm
        pose.orientation.w = qw / norm

        return pose

    def is_pose_within_workspace(self, x, y, z):
        x_min = self.x_min_input.value()
        x_max = self.x_max_input.value()
        y_min = self.y_min_input.value()
        y_max = self.y_max_input.value()
        z_min = self.z_min_input.value()
        z_max = self.z_max_input.value()

        if x_min > x_max or y_min > y_max or z_min > z_max:
            self.workspace_status_label.setText("Invalid workspace limits.")
            return False

        within_limits = (
            x_min <= x <= x_max and
            y_min <= y <= y_max and
            z_min <= z <= z_max
        )

        if within_limits:
            self.workspace_status_label.setText("Goal is inside safe workspace limits.")
            return True

        self.workspace_status_label.setText("Goal rejected: outside safe workspace limits.")
        return False

    def create_pick_pose_from_inputs(self):
        pose = Pose()

        pose.position.x = self.goal_x_input.value()
        pose.position.y = self.goal_y_input.value()
        pose.position.z = self.goal_z_input.value()

        pose.orientation.x = self.qx_input.value()
        pose.orientation.y = self.qy_input.value()
        pose.orientation.z = self.qz_input.value()
        pose.orientation.w = self.qw_input.value()

        return self.normalize_pose_quaternion(pose)

    def create_place_pose_from_inputs(self):
        pose = Pose()

        pose.position.x = self.place_x_input.value()
        pose.position.y = self.place_y_input.value()
        pose.position.z = self.place_z_input.value()

        pose.orientation.x = self.place_qx_input.value()
        pose.orientation.y = self.place_qy_input.value()
        pose.orientation.z = self.place_qz_input.value()
        pose.orientation.w = self.place_qw_input.value()

        return self.normalize_pose_quaternion(pose)

    def extract_detection_position(self, detection):
        if not isinstance(detection, dict):
            return None

        if isinstance(detection.get("position"), dict):
            position = detection["position"]
            if all(k in position for k in ["x", "y", "z"]):
                return float(position["x"]), float(position["y"]), float(position["z"])

        if all(k in detection for k in ["tag_x_m", "tag_y_m", "tag_z_m"]):
            return float(detection["tag_x_m"]), float(detection["tag_y_m"]), float(detection["tag_z_m"])

        if all(k in detection for k in ["x", "y", "z"]):
            return float(detection["x"]), float(detection["y"]), float(detection["z"])

        return None

    def send_pick_goal_only(self):
        pose = self.create_pick_pose_from_inputs()

        if not self.is_pose_within_workspace(pose.position.x, pose.position.y, pose.position.z):
            self.append_log("Pick pose rejected because it is outside workspace limits.")
            return

        self.ros_node.publish_goal(pose)
        self.last_command_label.setText("Last Command: Pick pose sent")
        self.append_log(
            f"Pick pose sent to /ur3_goal_pose: x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, z={pose.position.z:.3f}"
        )

    def send_place_goal_only(self):
        pose = self.create_place_pose_from_inputs()

        if not self.is_pose_within_workspace(pose.position.x, pose.position.y, pose.position.z):
            self.append_log("Place pose rejected because it is outside workspace limits.")
            return

        self.ros_node.publish_place_goal(pose)
        self.last_command_label.setText("Last Command: Place pose sent")
        self.append_log(
            f"Place pose sent to /plan_goal_pose: x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, z={pose.position.z:.3f}"
        )

    def start_system_with_goal(self):
        place_pose = self.create_place_pose_from_inputs()

        if not self.is_pose_within_workspace(place_pose.position.x, place_pose.position.y, place_pose.position.z):
            self.append_log("Start rejected: place pose outside workspace limits.")
            return

        self.ros_node.publish_place_goal(place_pose)
        self.ros_node.publish_pick_and_place_trigger()
        self.ros_node.publish_command("start")

        self.last_command_label.setText("Last Command: Pick-and-place triggered")
        self.append_log("Start pressed. Published /plan_goal_pose and /trigger_pick_and_place.")
        self.update_state_display("Executing", "Pick-and-place trigger sent.")

    def update_gripper_label(self):
        width = self.gripper_slider.value() / 1000.0
        self.gripper_width_label.setText(f"Width: {width:.3f} m")

    def send_gripper_command(self):
        width = self.gripper_slider.value() / 1000.0

        self.ros_node.publish_gripper_width(width)
        self.ros_node.publish_command("gripper")

        self.append_log(f"Published gripper width: {width:.3f} m")
        self.last_command_label.setText("Last Command: Gripper command sent")

    def update_velocity_label(self):
        velocity_scale = self.velocity_slider.value() / 100.0
        self.velocity_label.setText(f"Velocity Scale: {velocity_scale:.2f}")

    def send_velocity_scale(self):
        velocity_scale = self.velocity_slider.value() / 100.0

        self.ros_node.publish_velocity_scale(velocity_scale)
        self.append_log(f"Published velocity scale: {velocity_scale:.2f}")
        self.last_command_label.setText("Last Command: Velocity scale sent")

    def stop_system(self):
        self.ros_node.publish_command("stop")
        self.last_command_label.setText("Last Command: Stop")
        self.append_log("Stop command sent.")
        self.update_state_display("Stopped", "Stop requested.")

    def reset_system(self):
        self.ros_node.publish_command("reset")
        self.last_command_label.setText("Last Command: Reset")
        self.append_log("Reset command sent.")
        self.update_state_display("Idle", "System reset to idle state.")

    def refresh_camera_feed(self):
        frame = self.ros_node.latest_camera_frame

        if self.ros_node.latest_camera_error:
            self.camera_label.setText("Camera feed error:\n" + self.ros_node.latest_camera_error)
            return

        if frame is None:
            return

        try:
            rgb_frame = frame[:, :, ::-1].copy()
            height, width, channels = rgb_frame.shape
            bytes_per_line = channels * width

            qt_image = QImage(
                rgb_frame.data,
                width,
                height,
                bytes_per_line,
                QImage.Format_RGB888
            )

            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                self.camera_label.width(),
                self.camera_label.height(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            self.camera_label.setPixmap(scaled_pixmap)
            self.camera_topic_label.setText(
                f"Topic: {self.ros_node.camera_topic} | Resolution: {width} x {height}"
            )

        except Exception as error:
            self.camera_label.setText(f"Camera display error:\n{error}")

    def refresh_detection_display(self):
        if self.ros_node.latest_detection_error:
            self.detection_count_label.setText("Detected Objects: error")
            self.selected_detection_label.setText(
                "Detection parse error:\n" + self.ros_node.latest_detection_error
            )
            return

        detections = self.ros_node.latest_detection_result
        self.detection_count_label.setText(f"Detected Objects: {len(detections)}")

        current_index = self.detection_selector.currentIndex()

        self.detection_selector.blockSignals(True)
        self.detection_selector.clear()

        for i, detection in enumerate(detections):
            colour = detection.get("colour", detection.get("color", "Object")) if isinstance(detection, dict) else "Object"
            shape = detection.get("shape", "") if isinstance(detection, dict) else ""
            label = f"{i + 1}: {colour} {shape}".strip()
            self.detection_selector.addItem(label)

        self.detection_selector.blockSignals(False)

        if len(detections) == 0:
            self.selected_detection_label.setText("Selected Detection:\nNo valid object detected.")
            return

        if current_index < 0 or current_index >= len(detections):
            current_index = 0

        self.detection_selector.setCurrentIndex(current_index)
        self.update_selected_detection_display()

    def get_selected_detection(self):
        detections = self.ros_node.latest_detection_result

        if not detections:
            return None

        index = self.detection_selector.currentIndex()

        if index < 0 or index >= len(detections):
            return detections[0]

        return detections[index]

    def update_selected_detection_display(self):
        detection = self.get_selected_detection()

        if detection is None:
            self.selected_detection_label.setText("Selected Detection:\nNo object detected yet.")
            return

        position = self.extract_detection_position(detection)

        if isinstance(detection, dict):
            colour = detection.get("colour", detection.get("color", "Unknown"))
            shape = detection.get("shape", "Unknown")
            centroid_x = detection.get("centroid_x", "N/A")
            centroid_y = detection.get("centroid_y", "N/A")
            depth_m = detection.get("depth_m", None)
        else:
            colour = "Unknown"
            shape = "Unknown"
            centroid_x = "N/A"
            centroid_y = "N/A"
            depth_m = None

        text = f"Selected Detection:\nColour: {colour}\nShape: {shape}\nPixel: ({centroid_x}, {centroid_y})"

        if depth_m is not None:
            text += f"\nDepth: {float(depth_m):.3f} m"

        if position is not None:
            x, y, z = position
            text += f"\nPickup position: x={x:.3f}, y={y:.3f}, z={z:.3f} m"
        else:
            text += "\nPickup position: not available"

        self.selected_detection_label.setText(text)

    def use_selected_detection_as_pick_pose(self):
        detection = self.get_selected_detection()

        if detection is None:
            self.append_log("No detection available.")
            return

        position = self.extract_detection_position(detection)

        if position is None:
            self.append_log("Detection missing usable position data.")
            self.update_state_display("Idle", "Detection missing usable pickup coordinates.")
            return

        x, y, z = position

        self.goal_x_input.setValue(float(x))
        self.goal_y_input.setValue(float(y))
        self.goal_z_input.setValue(float(z))

        self.append_log(f"Copied detection to pick pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        self.last_command_label.setText("Last Command: Detection copied to pick pose")

    def refresh_motion_status(self):
        status = self.ros_node.latest_motion_status.strip().lower()

        if not status or status == self.last_processed_motion_status:
            return

        self.last_processed_motion_status = status
        self.motion_status_label.setText(f"Motion Status: {status}")

        if status == "idle":
            self.motion_stage_label.setText("Stage: Waiting for command")
            self.update_state_display("Idle", "Motion planner is idle.")
        elif status == "planning":
            self.motion_stage_label.setText("Stage: Generating trajectory")
            self.update_state_display("Executing", "Motion planner is generating trajectory.")
        elif status in ["moving", "executing"]:
            self.motion_stage_label.setText("Stage: Executing trajectory")
            self.update_state_display("Executing", "Robot is moving.")
        elif status == "completed":
            self.motion_stage_label.setText("Stage: Task completed")
            self.update_state_display("Completed", "Motion completed successfully.")
        elif status in ["failed", "stopped"]:
            self.motion_stage_label.setText("Stage: Task stopped/failed")
            self.update_state_display("Stopped", "Motion stopped or failed.")
        elif status == "busy":
            self.motion_stage_label.setText("Stage: Busy")
            self.update_state_display("Executing", "Motion planner is busy.")
        else:
            self.motion_stage_label.setText(f"Stage: {status}")
            self.update_state_display("Executing", f"Received motion status: {status}")

        self.append_log(f"Motion planner status: {status}")

    def closeEvent(self, event):
        try:
            self.append_log("Closing GUI and shutting down ROS2.")
        except Exception:
            pass

        try:
            if self.ros_node is not None:
                self.ros_node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = RobotGUI()
    window.show()

    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()