# Subsystem 3 — Interaction & Execution GUI (ROS2)

## Overview
This subsystem provides a PyQt + ROS2 GUI for human–robot interaction and execution control for a UR3 pick-and-place system.

The GUI integrates user control, perception outputs, and motion planning communication into a single interface.

---

# Features

## GUI Controls
- Start / Stop / Reset task controls  
- Cartesian goal pose input  
- Object placement position input  
- Gripper finger width slider  
- Velocity scaling slider  
- Safe workspace limit validation  

## Perception Integration
- Live RGB camera feed from Intel RealSense  
- Shape and colour detections displayed in GUI  
- AprilTag detections received from perception subsystem  
- Detection results can auto-populate pick pose targets  

## Motion Planning Integration

### Publishes to:
```text
/ur3_goal_pose
/place_goal_pose
/finger_width_controller/commands
/velocity_scale
/gui_command
```

### Subscribes to:
```text
/motion_status
/detection_result
/camera/camera/color/image_raw
```

---

# Workspace Build

From workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select gui_control
source install/setup.bash
```

---

# Dependencies

## GUI + Python
```bash
sudo apt update
sudo apt install python3-pyqt5 python3-opencv python3-pip
```

## ROS Humble cv_bridge compatibility fix
Required:

```bash
pip uninstall numpy opencv-python-headless -y
pip install numpy==1.26.4
pip install opencv-python-headless==4.8.1.78
```

---

# Full System Bring-Up

## Terminal 1 — Launch Intel RealSense
```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

---

## Terminal 2 — Shape and Colour Detection
```bash
cd ~/ros2_ws
source install/setup.bash

ros2 run rs2_snl_pkg shape_colour_detector_node
```

Publishes:

```text
/detection_result
```

---

## Terminal 3 — AprilTag Detection
```bash
cd ~/ros2_ws
source install/setup.bash

ros2 run apriltag_ros apriltag_node \
--ros-args \
-r image_rect:=/camera/camera/color/image_raw \
-r camera_info:=/camera/camera/color/camera_info \
--params-file ~/ros2_ws/src/RS2_SNL/rs2_snl_pkg/config/apriltag.yaml
```

---

## Terminal 4 — Launch GUI
```bash
cd ~/ros2_ws
source install/setup.bash

ros2 run gui_control gui_node
```

---

# Optional Full UR3 Integration
Launch motion planner / UR3 simulation:

```bash
ros2 launch ur3_motion_planner bringup.launch.py
```

Full pipeline:

```text
Perception
→ GUI
→ Motion Planner
→ UR3 Simulation
```

---

# Topic Verification

Check active topics:

```bash
ros2 topic list
```

Expected:

```text
/camera/camera/color/image_raw
/detection_result
/ur3_goal_pose
/place_goal_pose
/finger_width_controller/commands
/velocity_scale
/gui_command
/motion_status
```

---

# Testing

## Test Pick Pose Publisher
```bash
ros2 topic echo /ur3_goal_pose
```

Press:

```text
Send Pick Pose
```

---

## Test Place Pose Publisher
```bash
ros2 topic echo /place_goal_pose
```

Press:

```text
Send Place Pose
```

---

## Test Gripper Command
```bash
ros2 topic echo /finger_width_controller/commands
```

Use gripper slider + send command.

---

## Test Velocity Scale
```bash
ros2 topic echo /velocity_scale
```

Adjust slider and publish.

---

## Test Detection Integration

1. Detection objects appear in GUI dropdown  
2. Select object  
3. Click:

```text
Use Detection as Pick Pose
```

This copies:

```text
tag_x_m
tag_y_m
tag_z_m
```

into the Cartesian goal pose.

---

## Test Full Task

Press:

```text
Start Pick-and-Place
```

Publishes:

```text
start
```

to:

```text
/ gui_command
```

and triggers execution.

---

# Rebuild After Code Changes
```bash
cd ~/ros2_ws
colcon build --packages-select gui_control
source install/setup.bash
```

---

# Files

Main GUI node:

```text
gui_control/gui_node.py
```

---

# Rubric Coverage

Implemented:

✅ GUI includes object placement positions  
✅ User can set maximum/minimum velocity using slider  
✅ User can set safe workspace positions/limits in GUI  
✅ Live camera feed displayed through PyQt GUI  
✅ Perception subsystem integration  
✅ GUI ↔ motion planning subsystem integration  

---

# Bring-Up Order (Quick Reference)

```text
1. Launch RealSense
2. Run shape/colour detector
3. Run AprilTag node
4. Launch GUI
5. Test pick pose, detections, gripper and camera feed
```