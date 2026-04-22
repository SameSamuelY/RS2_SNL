#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('ur3_goal_gui')
        self.publisher = self.create_publisher(Pose, '/ur3_goal_pose', 10)
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
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: pos({x},{y},{z}) quat({qx},{qy},{qz},{qw})")

class Application:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("UR3 Goal Sender")
        self.root.geometry("400x350")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Position frame
        pos_frame = ttk.LabelFrame(root, text="Position (meters)")
        pos_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(pos_frame, text="X:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.x_entry = ttk.Entry(pos_frame, width=10)
        self.x_entry.insert(0, "0.1")
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(pos_frame, text="Y:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.y_entry = ttk.Entry(pos_frame, width=10)
        self.y_entry.insert(0, "0.4")
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(pos_frame, text="Z:").grid(row=0, column=4, padx=5, pady=5, sticky="e")
        self.z_entry = ttk.Entry(pos_frame, width=10)
        self.z_entry.insert(0, "0.2")
        self.z_entry.grid(row=0, column=5, padx=5, pady=5)

        # Orientation frame (quaternion)
        quat_frame = ttk.LabelFrame(root, text="Orientation (quaternion)")
        quat_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(quat_frame, text="qx:").grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.qx_entry = ttk.Entry(quat_frame, width=8)
        self.qx_entry.insert(0, "0.707")
        self.qx_entry.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(quat_frame, text="qy:").grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.qy_entry = ttk.Entry(quat_frame, width=8)
        self.qy_entry.insert(0, "0.707")
        self.qy_entry.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(quat_frame, text="qz:").grid(row=0, column=4, padx=5, pady=5, sticky="e")
        self.qz_entry = ttk.Entry(quat_frame, width=8)
        self.qz_entry.insert(0, "0.0")
        self.qz_entry.grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(quat_frame, text="qw:").grid(row=0, column=6, padx=5, pady=5, sticky="e")
        self.qw_entry = ttk.Entry(quat_frame, width=8)
        self.qw_entry.insert(0, "0.0")
        self.qw_entry.grid(row=0, column=7, padx=5, pady=5)

        # Send button
        self.send_button = ttk.Button(root, text="Send Goal", command=self.send_goal)
        self.send_button.pack(pady=15)

        # Status label
        self.status = ttk.Label(root, text="Ready")
        self.status.pack()

    def send_goal(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            qx = float(self.qx_entry.get())
            qy = float(self.qy_entry.get())
            qz = float(self.qz_entry.get())
            qw = float(self.qw_entry.get())

            # Normalize quaternion (optional but good)
            norm = (qx*qx + qy*qy + qz*qz + qw*qw)**0.5
            if norm != 0:
                qx /= norm; qy /= norm; qz /= norm; qw /= norm

            self.node.publish_pose(x, y, z, qx, qy, qz, qw)
            self.status.config(text=f"Sent: ({x},{y},{z}) / ({qx},{qy},{qz},{qw})")
        except ValueError:
            messagebox.showerror("Input error", "Please enter valid numbers for all fields.")
            self.status.config(text="Invalid input")

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