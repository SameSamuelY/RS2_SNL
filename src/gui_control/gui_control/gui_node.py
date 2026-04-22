#!/usr/bin/env python3

import sys
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from PyQt5.QtCore import Qt, QTimer
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
)


class GuiROSNode(Node):
    def __init__(self):
        super().__init__('gui_controller')

        # Publish target pose to motion planning subsystem
        self.goal_publisher = self.create_publisher(Pose, '/ur3_goal_pose', 10)

        # Publish optional high-level GUI commands
        self.command_publisher = self.create_publisher(String, '/gui_command', 10)

        # Subscribe to motion planning feedback
        self.motion_status_subscriber = self.create_subscription(
            String,
            '/motion_status',
            self.motion_status_callback,
            10
        )

        self.latest_motion_status = "idle"

        self.get_logger().info("GUI ROS node ready")

    def publish_goal(self, pose: Pose):
        self.goal_publisher.publish(pose)
        self.get_logger().info(
            f"Published goal pose: "
            f"x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, "
            f"z={pose.position.z:.3f}"
        )

    def publish_command(self, command_text: str):
        msg = String()
        msg.data = command_text
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published GUI command: {command_text}")

    def motion_status_callback(self, msg: String):
        self.latest_motion_status = msg.data
        self.get_logger().info(f"Received motion status: {msg.data}")


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
        self.setGeometry(200, 120, 1100, 700)
        self.setMinimumSize(950, 620)

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
                font-size: 18px;
                font-weight: bold;
                color: #1f2d3d;
            }

            QLabel#StateValue {
                font-size: 18px;
                font-weight: bold;
                padding: 8px 14px;
                border-radius: 8px;
                background-color: #e9ecef;
                color: #212529;
            }

            QLabel#InfoLabel {
                font-size: 13px;
                color: #344054;
                padding: 2px 0;
            }

            QPushButton {
                border: none;
                border-radius: 10px;
                padding: 12px;
                font-size: 14px;
                font-weight: bold;
                min-height: 42px;
            }

            QPushButton#StartButton {
                background-color: #2e7d32;
                color: white;
            }

            QPushButton#StartButton:hover {
                background-color: #256628;
            }

            QPushButton#StopButton {
                background-color: #c62828;
                color: white;
            }

            QPushButton#StopButton:hover {
                background-color: #a61f1f;
            }

            QPushButton#ResetButton {
                background-color: #1565c0;
                color: white;
            }

            QPushButton#ResetButton:hover {
                background-color: #0f4f98;
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

            QDoubleSpinBox {
                background-color: #ffffff;
                border: 1px solid #d0d5dd;
                border-radius: 6px;
                padding: 4px;
                min-height: 28px;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        content_layout = self.build_content()
        main_layout.addLayout(content_layout)

        central_widget.setLayout(main_layout)

    def init_ros_timers(self):
        self.ros_spin_timer = QTimer()
        self.ros_spin_timer.timeout.connect(self.spin_ros_once)
        self.ros_spin_timer.start(20)

        self.status_refresh_timer = QTimer()
        self.status_refresh_timer.timeout.connect(self.refresh_motion_status)
        self.status_refresh_timer.start(100)

    def spin_ros_once(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def build_content(self):
        content_layout = QHBoxLayout()
        content_layout.setSpacing(15)

        left_panel = QVBoxLayout()
        left_panel.setSpacing(15)

        right_panel = QVBoxLayout()
        right_panel.setSpacing(15)

        left_panel.addWidget(self.build_control_group(), 0)
        left_panel.addWidget(self.build_goal_group(), 0)
        left_panel.addWidget(self.build_status_group(), 0)
        left_panel.addWidget(self.build_motion_group(), 0)
        left_panel.addWidget(self.build_log_group(), 1)

        right_panel.addWidget(self.build_camera_group(), 1)
        right_panel.addWidget(self.build_info_group(), 0)

        content_layout.addLayout(left_panel, 1)
        content_layout.addLayout(right_panel, 1)

        return content_layout

    def build_control_group(self):
        group = QGroupBox("System Controls")
        layout = QVBoxLayout()
        layout.setSpacing(12)

        instruction = QLabel(
            "Use the buttons below to control the subsystem. "
            "Start publishes a target pose to motion planning."
        )
        instruction.setWordWrap(True)
        instruction.setObjectName("InfoLabel")

        self.start_button = QPushButton("Start")
        self.start_button.setObjectName("StartButton")

        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("StopButton")

        self.reset_button = QPushButton("Reset")
        self.reset_button.setObjectName("ResetButton")

        self.start_button.clicked.connect(self.start_system_with_goal)
        self.stop_button.clicked.connect(self.stop_system)
        self.reset_button.clicked.connect(self.reset_system)

        button_layout = QHBoxLayout()
        button_layout.setSpacing(10)
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.reset_button)

        layout.addWidget(instruction)
        layout.addLayout(button_layout)
        group.setLayout(layout)

        return group

    def build_goal_group(self):
        group = QGroupBox("Goal Pose Input")
        layout = QGridLayout()
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(10)

        x_label = QLabel("Goal X (m):")
        x_label.setObjectName("InfoLabel")
        self.goal_x_input = QDoubleSpinBox()
        self.goal_x_input.setRange(-1.0, 1.0)
        self.goal_x_input.setSingleStep(0.01)
        self.goal_x_input.setDecimals(3)
        self.goal_x_input.setValue(0.300)

        y_label = QLabel("Goal Y (m):")
        y_label.setObjectName("InfoLabel")
        self.goal_y_input = QDoubleSpinBox()
        self.goal_y_input.setRange(-1.0, 1.0)
        self.goal_y_input.setSingleStep(0.01)
        self.goal_y_input.setDecimals(3)
        self.goal_y_input.setValue(-0.200)

        z_label = QLabel("Goal Z (m):")
        z_label.setObjectName("InfoLabel")
        self.goal_z_input = QDoubleSpinBox()
        self.goal_z_input.setRange(0.0, 1.5)
        self.goal_z_input.setSingleStep(0.01)
        self.goal_z_input.setDecimals(3)
        self.goal_z_input.setValue(0.500)

        note_label = QLabel(
            "These values are sent as a Pose to /ur3_goal_pose when Start is pressed."
        )
        note_label.setWordWrap(True)
        note_label.setObjectName("InfoLabel")

        layout.addWidget(x_label, 0, 0)
        layout.addWidget(self.goal_x_input, 0, 1)

        layout.addWidget(y_label, 1, 0)
        layout.addWidget(self.goal_y_input, 1, 1)

        layout.addWidget(z_label, 2, 0)
        layout.addWidget(self.goal_z_input, 2, 1)

        layout.addWidget(note_label, 3, 0, 1, 2)

        group.setLayout(layout)
        return group

    def build_status_group(self):
        group = QGroupBox("System Status")
        layout = QGridLayout()
        layout.setHorizontalSpacing(15)
        layout.setVerticalSpacing(10)

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
        layout.setSpacing(8)

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
        self.log_box.setPlaceholderText("System logs will appear here...")

        layout.addWidget(self.log_box)
        group.setLayout(layout)

        self.append_log("GUI initialised successfully.")
        self.append_log("ROS2 node initialised.")
        self.append_log("Waiting for motion planner feedback on /motion_status.")

        return group

    def build_camera_group(self):
        group = QGroupBox("Camera Feed")
        layout = QVBoxLayout()

        camera_frame = QFrame()
        camera_frame.setObjectName("CameraFrame")
        camera_frame.setMinimumHeight(320)
        camera_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        frame_layout = QVBoxLayout()
        frame_layout.setAlignment(Qt.AlignCenter)

        camera_placeholder = QLabel(
            "Live Camera Feed Placeholder\n\n"
            "(Connect perception image topic here later)"
        )
        camera_placeholder.setObjectName("CameraPlaceholder")
        camera_placeholder.setAlignment(Qt.AlignCenter)

        frame_layout.addWidget(camera_placeholder)
        camera_frame.setLayout(frame_layout)

        layout.addWidget(camera_frame)
        group.setLayout(layout)

        return group

    def build_info_group(self):
        group = QGroupBox("Subsystem Information")
        layout = QVBoxLayout()
        layout.setSpacing(8)

        self.mode_label = QLabel("Mode: Manual GUI Control")
        self.mode_label.setObjectName("InfoLabel")

        self.connection_label = QLabel("ROS2 Connection: Active")
        self.connection_label.setObjectName("InfoLabel")

        self.last_command_label = QLabel("Last Command: None")
        self.last_command_label.setObjectName("InfoLabel")

        self.integration_label = QLabel("Integration: Publishing Pose goals to motion planning")
        self.integration_label.setObjectName("InfoLabel")

        layout.addWidget(self.mode_label)
        layout.addWidget(self.connection_label)
        layout.addWidget(self.last_command_label)
        layout.addWidget(self.integration_label)

        group.setLayout(layout)
        return group

    def append_log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_box.append(f"[{timestamp}] {message}")

    def update_state_display(self, state, message):
        self.current_state = state
        self.state_value.setText(state)
        self.status_message.setText(message)

        if state == "Executing":
            self.state_value.setStyleSheet("""
                font-size: 18px;
                font-weight: bold;
                padding: 8px 14px;
                border-radius: 8px;
                background-color: #e8f5e9;
                color: #2e7d32;
            """)
        elif state == "Stopped":
            self.state_value.setStyleSheet("""
                font-size: 18px;
                font-weight: bold;
                padding: 8px 14px;
                border-radius: 8px;
                background-color: #fdecea;
                color: #c62828;
            """)
        elif state == "Completed":
            self.state_value.setStyleSheet("""
                font-size: 18px;
                font-weight: bold;
                padding: 8px 14px;
                border-radius: 8px;
                background-color: #e3f2fd;
                color: #1565c0;
            """)
        else:
            self.state_value.setStyleSheet("""
                font-size: 18px;
                font-weight: bold;
                padding: 8px 14px;
                border-radius: 8px;
                background-color: #e9ecef;
                color: #212529;
            """)

    def start_system_with_goal(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x_input.value()
        goal_pose.position.y = self.goal_y_input.value()
        goal_pose.position.z = self.goal_z_input.value()

        # Neutral orientation for simple testing
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0
        goal_pose.orientation.w = 1.0

        self.ros_node.publish_goal(goal_pose)
        self.ros_node.publish_command("start")

        self.last_command_label.setText("Last Command: Start (goal published)")
        self.append_log(
            f"Published goal pose to /ur3_goal_pose: "
            f"x={goal_pose.position.x:.3f}, "
            f"y={goal_pose.position.y:.3f}, "
            f"z={goal_pose.position.z:.3f}"
        )

        self.update_state_display(
            "Executing",
            "Goal pose sent to motion planning subsystem."
        )

    def stop_system(self):
        self.ros_node.publish_command("stop")
        self.last_command_label.setText("Last Command: Stop")
        self.append_log("Stop button pressed. Published command: stop")

        self.update_state_display(
            "Stopped",
            "Stop requested. Motion planner should handle cancellation if implemented."
        )

    def reset_system(self):
        self.ros_node.publish_command("reset")
        self.last_command_label.setText("Last Command: Reset")
        self.append_log("Reset button pressed. Published command: reset")

        self.update_state_display("Idle", "System reset to idle state.")

    def refresh_motion_status(self):
        status = self.ros_node.latest_motion_status.strip().lower()

        if not status or status == self.last_processed_motion_status:
            return

        self.last_processed_motion_status = status
        self.motion_status_label.setText(f"Motion Status: {status}")

        if status == "idle":
            self.motion_stage_label.setText("Stage: Waiting for command")
            self.update_state_display("Idle", "Motion planner is idle.")
            self.append_log("Motion planner status: idle")

        elif status == "planning":
            self.motion_stage_label.setText("Stage: Generating trajectory")
            self.update_state_display("Executing", "Motion planner is generating a trajectory.")
            self.append_log("Motion planner status: planning")

        elif status == "moving":
            self.motion_stage_label.setText("Stage: Executing trajectory")
            self.update_state_display("Executing", "Robot is moving to the target pose.")
            self.append_log("Motion planner status: moving")

        elif status == "moving_to_pick":
            self.motion_stage_label.setText("Stage: Moving to pick position")
            self.update_state_display("Executing", "Robot is moving to the pick position.")
            self.append_log("Motion planner status: moving_to_pick")

        elif status == "grasping":
            self.motion_stage_label.setText("Stage: Grasping object")
            self.update_state_display("Executing", "Robot is attempting to grasp the object.")
            self.append_log("Motion planner status: grasping")

        elif status == "moving_to_place":
            self.motion_stage_label.setText("Stage: Moving to place position")
            self.update_state_display("Executing", "Robot is moving to the place position.")
            self.append_log("Motion planner status: moving_to_place")

        elif status == "returning_home":
            self.motion_stage_label.setText("Stage: Returning home")
            self.update_state_display("Executing", "Robot is returning to the home pose.")
            self.append_log("Motion planner status: returning_home")

        elif status == "completed":
            self.motion_stage_label.setText("Stage: Task completed")
            self.update_state_display("Completed", "Motion planner completed the task successfully.")
            self.append_log("Motion planner status: completed")

        elif status == "failed":
            self.motion_stage_label.setText("Stage: Task failed")
            self.update_state_display("Stopped", "Motion planner reported task failure.")
            self.append_log("Motion planner status: failed")

        elif status == "stopped":
            self.motion_stage_label.setText("Stage: Execution stopped")
            self.update_state_display("Stopped", "Motion planner stopped execution.")
            self.append_log("Motion planner status: stopped")

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
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()