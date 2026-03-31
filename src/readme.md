# RS2 GUI Control (Subsystem 3 – Interaction and Execution)

This project implements a **ROS 2 Humble + PyQt5 GUI** for controlling a robotic system.  
It is part of **Subsystem 3: Interaction and Execution**, focusing on user interaction, system control, and command communication.

---

## 🚀 Features

- Start / Stop / Reset control buttons  
- System state display (Idle, Executing, Stopped, Completed)  
- Command logging interface  
- ROS 2 publisher for command communication (`/gui_command`)  
- Ready for integration with motion planning subsystem  

---

## 📁 Project Structure

```bash
ros2_ws/
├── src/
│   └── gui_control/
│       ├── gui_control/
│       │   ├── __init__.py
│       │   └── gui_node.py
│       ├── resource/
│       │   └── gui_control
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── build/        # generated (DO NOT COMMIT)
├── install/      # generated (DO NOT COMMIT)
├── log/          # generated (DO NOT COMMIT)
└── README.md

## How to Run

Follow these steps to run the GUI.

### 1. Open a terminal and go to your workspace
```bash
cd ~/ros2_ws

### 2. Source ROS 2
source /opt/ros/humble/setup.bash

### 3. Build the package (only needed if first time or after changes)
colcon build --packages-select gui_control --symlink-install

###4. Source the workspace
source install/setup.bash

###5. Run the GUI
ros2 run gui_control gui_node

The GUI window should appear.

(Optional) Check ROS2 Communication

Open another terminal and run:

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /gui_command

###When you press buttons in the GUI, you should see:
data: start
data: stop
data: reset

###Notes
Always run commands from ~/ros2_ws
Always source ROS2 and the workspace before running
Rebuild only when you modify code
Make sure PyQt5 is installed:
sudo apt install python3-pyqt5
Do not commit generated folders (build/, install/, log/)