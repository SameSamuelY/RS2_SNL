import sys
from datetime import datetime

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
)


class RobotGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.current_state = "Idle"
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Subsystem 3 - Interaction and Execution")
        self.setGeometry(200, 120, 1000, 650)
        self.setMinimumSize(900, 600)

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

            QLabel#TitleLabel {
                font-size: 24px;
                font-weight: bold;
                color: #1f2d3d;
            }

            QLabel#SubtitleLabel {
                font-size: 13px;
                color: #5b6570;
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
        """)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        header_layout = self.build_header()
        content_layout = self.build_content()

        main_layout.addLayout(header_layout)
        main_layout.addLayout(content_layout)

        self.setLayout(main_layout)

    def build_header(self):
        header_layout = QVBoxLayout()
        header_layout.setSpacing(4)

        title_label = QLabel("Robot Control Panel")
        title_label.setObjectName("TitleLabel")
        title_label.setAlignment(Qt.AlignCenter)

        subtitle_label = QLabel("Subsystem 3 - Interaction and Execution")
        subtitle_label.setObjectName("SubtitleLabel")
        subtitle_label.setAlignment(Qt.AlignCenter)

        header_layout.addWidget(title_label)
        header_layout.addWidget(subtitle_label)

        return header_layout

    def build_content(self):
        content_layout = QHBoxLayout()
        content_layout.setSpacing(15)

        left_panel = QVBoxLayout()
        left_panel.setSpacing(15)

        right_panel = QVBoxLayout()
        right_panel.setSpacing(15)

        left_panel.addWidget(self.build_control_group(), 0)
        left_panel.addWidget(self.build_status_group(), 0)
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

        instruction = QLabel("Use the buttons below to control the subsystem execution.")
        instruction.setWordWrap(True)
        instruction.setObjectName("InfoLabel")

        self.start_button = QPushButton("Start")
        self.start_button.setObjectName("StartButton")
        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("StopButton")
        self.reset_button = QPushButton("Reset")
        self.reset_button.setObjectName("ResetButton")

        self.start_button.clicked.connect(self.start_system)
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

    def build_log_group(self):
        group = QGroupBox("Command Log")
        layout = QVBoxLayout()

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setPlaceholderText("System logs will appear here...")

        layout.addWidget(self.log_box)
        group.setLayout(layout)

        self.append_log("GUI initialised successfully.")
        self.append_log("System state set to Idle.")

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

        camera_placeholder = QLabel("Live Camera Feed Placeholder\n\n(Connect perception output here later)")
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

        self.connection_label = QLabel("Integration Status: Not Connected to ROS Yet")
        self.connection_label.setObjectName("InfoLabel")

        self.last_command_label = QLabel("Last Command: None")
        self.last_command_label.setObjectName("InfoLabel")

        layout.addWidget(self.mode_label)
        layout.addWidget(self.connection_label)
        layout.addWidget(self.last_command_label)

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

    def start_system(self):
        self.update_state_display("Executing", "System is currently running the task.")
        self.last_command_label.setText("Last Command: Start")
        self.append_log("Start button pressed.")
        self.append_log("Execution command triggered.")

        QTimer.singleShot(3000, self.complete_task_demo)

    def stop_system(self):
        self.update_state_display("Stopped", "System execution has been stopped by the user.")
        self.last_command_label.setText("Last Command: Stop")
        self.append_log("Stop button pressed.")
        self.append_log("Execution halted safely.")

    def reset_system(self):
        self.update_state_display("Idle", "System is waiting for user input.")
        self.last_command_label.setText("Last Command: Reset")
        self.append_log("Reset button pressed.")
        self.append_log("System returned to Idle state.")

    def complete_task_demo(self):
        if self.current_state == "Executing":
            self.update_state_display("Completed", "Task completed successfully.")
            self.last_command_label.setText("Last Command: Auto-complete demo")
            self.append_log("Demo task completed successfully.")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    window = RobotGUI()
    window.show()
    sys.exit(app.exec_())