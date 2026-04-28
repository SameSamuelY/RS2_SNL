# UR3 + OnRobot RG2 ROS 2 Workspace

This workspace provides a complete ROS 2 Humble environment for controlling a Universal Robots UR3 (CB3) equipped with an OnRobot RG2 gripper (using MoveIt Task Constructor (MTC) for advanced manipulation later). It supports both simulation (URSim in Docker) and real hardware.

**System Requirements:**

- OS: Ubuntu 22.04 (native or WSL2)
- ROS 2: Humble
- Workspace size: ~15 GB (including MoveIt Task Constructor source)
- Docker (for URSim)

# Installation:

1. Install ROS 2 Humble:
```bash
sudo apt update && sudo apt install ros-humble-desktop ros-humble-ros2-control ros-humble-ros2-controllers
```

2. Install MoveIt 2:
```bash
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools
```

3. Install MoveIt Task Constructor (source):
```bash
cd ~/git/RS2_SNL/ur_ws/src
git clone -b humble https://github.com/moveit/moveit_task_constructor.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

4. Clone this workspace (or your own copy):
```bash
cd ~/git/RS2_SNL
git clone <your-repo-url> ur_ws
```

5. Install additional dependencies:
```bash
cd ~/git/RS2_SNL/ur_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libnet1-dev
```

**Building the Workspace:**
```bash
cd ~/git/RS2_SNL/ur_ws
colcon build --symlink-install
source install/setup.bash
```
Note: The first build of moveit_task_constructor_core may take 10‑15 minutes.

# Launching the System:

With URSim (Docker)
Start URSim container first (see URSim instructions), 
```bash
ros2 run ur_client_library start_ursim.sh \
  -m ur3 \
  -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"
```

then:
```bash
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.56.101 \
    planner_id:=RRTConnectkConfigDefault \
    ignore_if_busy:=true \
    trajectory_velocity_scaling:=0.1 \
    trajectory_acceleration_scaling:=0.1 \
    connection_type:=serial
```

Real UR3 (without gripper)
All robot motion is simulated in RViz (no URSim needed).
```bash
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.0.195 \
    planner_id:=RRTConnectkConfigDefault \
    ignore_if_busy:=true \
    trajectory_velocity_scaling:=0.1 \
    trajectory_acceleration_scaling:=0.1 \
    connection_type:=serial
```


The launch file (bringup) starts:
- UR driver
- MoveIt & RViz
- Planner listener node (ur3_planner_listener)
- GUI (ur3_goal_gui.py)
- Gripper controller (finger_width_trajectory_controller)


# Controlling the Robot:

MoveIt RViz Interface:
- Interactive markers – drag to set goal pose, click Plan & Execute
- Joints tab – manually set joint angles
- MotionPlanning panel – plan, execute, add collision objects

Using the Planner Listener Node:
The node subscribes to ***/ur3_goal_pose (geometry_msgs/Pose)*** and executes Cartesian arm motions.

Publish a pose target:
```bash
ros2 topic pub --once /ur3_goal_pose geometry_msgs/msg/Pose \
  "{position: {x: -0.3, y: -0.2, z: 0.01}, orientation: {y: 0.707}}"
```

The node will:
- Wait for External Control program to be running (if using URSim)
- Activate scaled_joint_trajectory_controller
- Plan and execute the motion
- Publish status to /motion_status

Gripper Control:
The listener node also subscribes to ***/ur3_gripper_cmd (std_msgs/Float64MultiArray)***. Send a width in meters (0.0 = closed, 0.11 = fully open).
```bash
ros2 topic pub --once /ur3_gripper_cmd std_msgs/msg/Float64MultiArray "{data: [0.11]}" # fully open
ros2 topic pub --once /ur3_gripper_cmd std_msgs/msg/Float64MultiArray "{data: [0.0]}"  # fully close
```

The node will automatically activate finger_width_trajectory_controller and move the gripper to the requested width (clamped to joint limits).


**MoveIt Task Constructor (MTC)**

The mtc_node (in package ur3_mtc) demonstrates a multi‑stage task (after launching the robot system):
```bash
ros2 run ur3_mtc mtc_node
```

The arm will move to a predefined pose configuration and the gripper will open. The trajectory is visualized in RViz.


# Troubleshooting:

Controller activation warnings:
If you see "Controller with name 'scaled_joint_trajectory_controller' is not active", the listener node will automatically activate it. Wait a few seconds after launch.

Gripper does not move:
- Ensure finger_width_trajectory_controller is loaded and active:
  ros2 control list_controllers | grep finger
- If inactive, the listener node will activate it automatically on first gripper command.

MoveIt planning fails (Invalid start state):
- Check that the robot model in RViz is green (not red). If red, there is a collision (e.g., closed gripper colliding with ground). Publish an open command first.
- Ensure use_fake_hardware:=true if URSim is not running.

RViz freezes or grey cursor:
- Kill all RViz processes: killall rviz2
- Delete corrupt config: rm ~/.rviz/default.rviz
- Restart the system.