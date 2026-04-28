#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('ur3_goal_gui')
        self.pose_pub = self.create_publisher(Pose, '/ur3_goal_pose', 10)
        # self.gripper_pub = self.create_publisher(Float64MultiArray, '/finger_width_controller/commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/ur3_gripper_cmd', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.current_gripper_width = 0.0  # meters
        self.get_logger().info("GUI publisher node started")

    def publish_pose(self, x, y, z, qx, qy, qz, qw):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.pose_pub.publish(msg)
        self.get_logger().info(f"Published pose: ({x},{y},{z}) / ({qx},{qy},{qz},{qw})")

    def publish_gripper(self, width_m):
        msg = Float64MultiArray()
        msg.data = [width_m]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Published gripper width: {width_m} m")

    def joint_callback(self, msg):
        try:
            idx = msg.name.index('finger_width')
            self.current_gripper_width = msg.position[idx]
        except ValueError:
            pass  # finger_width not in joint list yet

class Application:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("UR3 Control Panel")
        self.root.geometry("450x500")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # === Pose control frame ===
        pose_frame = ttk.LabelFrame(root, text="Cartesian Goal (position in meters, orientation quaternion)")
        pose_frame.pack(fill="x", padx=10, pady=5)

        # Position row
        ttk.Label(pose_frame, text="X:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.x_entry = ttk.Entry(pose_frame, width=10)
        self.x_entry.insert(0, "-0.3")
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(pose_frame, text="Y:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.y_entry = ttk.Entry(pose_frame, width=10)
        self.y_entry.insert(0, "-0.2")
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(pose_frame, text="Z:").grid(row=0, column=4, padx=5, pady=5, sticky="e")
        self.z_entry = ttk.Entry(pose_frame, width=10)
        self.z_entry.insert(0, "0.01")
        self.z_entry.grid(row=0, column=5, padx=5, pady=5)

        # Quaternion row
        ttk.Label(pose_frame, text="qx:").grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.qx_entry = ttk.Entry(pose_frame, width=8)
        self.qx_entry.insert(0, "0.0")
        self.qx_entry.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(pose_frame, text="qy:").grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.qy_entry = ttk.Entry(pose_frame, width=8)
        self.qy_entry.insert(0, "0.707")
        self.qy_entry.grid(row=1, column=3, padx=5, pady=5)

        ttk.Label(pose_frame, text="qz:").grid(row=1, column=4, padx=5, pady=5, sticky="e")
        self.qz_entry = ttk.Entry(pose_frame, width=8)
        self.qz_entry.insert(0, "0.0")
        self.qz_entry.grid(row=1, column=5, padx=5, pady=5)

        ttk.Label(pose_frame, text="qw:").grid(row=1, column=6, padx=5, pady=5, sticky="e")
        self.qw_entry = ttk.Entry(pose_frame, width=8)
        self.qw_entry.insert(0, "0.0")
        self.qw_entry.grid(row=1, column=7, padx=5, pady=5)

        send_pose_btn = ttk.Button(pose_frame, text="Send Pose Goal", command=self.send_pose)
        send_pose_btn.grid(row=2, column=0, columnspan=8, pady=10)

        # === Gripper control frame ===
        gripper_frame = ttk.LabelFrame(root, text="Gripper Control (RG2, 0.0 - 0.11 m)")
        gripper_frame.pack(fill="x", padx=10, pady=5)

        self.gripper_slider = tk.Scale(gripper_frame, from_=0.0, to=0.11, resolution=0.01,
                                       orient=tk.HORIZONTAL, length=300, label="Width (m)")
        self.gripper_slider.set(0.05)  # default 50 mm
        self.gripper_slider.pack(pady=5)

        send_gripper_btn = ttk.Button(gripper_frame, text="Send Gripper Command", command=self.send_gripper)
        send_gripper_btn.pack(pady=5)

        # Status labels
        self.status_pose = ttk.Label(root, text="Pose status: Ready")
        self.status_pose.pack(pady=2)
        self.status_gripper = ttk.Label(root, text="Gripper status: unknown")
        self.status_gripper.pack(pady=2)

        # Start updating gripper status periodically
        self.update_gripper_status()

    def send_pose(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            qx = float(self.qx_entry.get())
            qy = float(self.qy_entry.get())
            qz = float(self.qz_entry.get())
            qw = float(self.qw_entry.get())

            # Normalize quaternion
            norm = (qx*qx + qy*qy + qz*qz + qw*qw)**0.5
            if norm != 0:
                qx /= norm; qy /= norm; qz /= norm; qw /= norm

            self.node.publish_pose(x, y, z, qx, qy, qz, qw)
            self.status_pose.config(text=f"Pose sent: ({x},{y},{z}) / ({qx:.2f},{qy:.2f},{qz:.2f},{qw:.2f})")
        except ValueError:
            messagebox.showerror("Input error", "Please enter valid numbers for all pose fields.")
            self.status_pose.config(text="Invalid pose input")

    def send_gripper(self):
        width = self.gripper_slider.get()
        self.node.publish_gripper(width)
        self.status_gripper.config(text=f"Gripper command sent: {width:.3f} m")

    def update_gripper_status(self):
        # Read current gripper width from the ROS node (thread‑safe)
        width = self.node.current_gripper_width
        self.status_gripper.config(text=f"Current gripper width: {width:.3f} m")
        # Schedule next update after 200 ms
        self.root.after(200, self.update_gripper_status)

    def on_closing(self):
        rclpy.shutdown()
        self.root.destroy()

def spin_node(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = GoalPublisher()

    # Start ROS spinning in a separate thread
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    # Start GUI
    root = tk.Tk()
    app = Application(root, node)
    root.mainloop()

if __name__ == "__main__":
    main()