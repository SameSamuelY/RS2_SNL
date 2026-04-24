cd ~/git/RS2_SNL/ur_ws/
# colcon build (all)
cd ~/git/RS2_SNL/ur_ws/
colcon build --symlink-install
source install/setup.bash

# colcon build (ur3 urdf)
cd ~/git/RS2_SNL/ur_ws/
colcon build --packages-select ur_description --symlink-install
source install/setup.bash

# colcon build (ur3_planner)
cd ~/git/RS2_SNL/ur_ws/
colcon build --packages-select ur3_planner --symlink-install
source install/setup.bash

# Source
source ~/git/RS2_SNL/ur_ws/install/setup.bash

# Docker 
http://localhost:6080/vnc.html

# Initialise docker (if not)
sudo service docker start

# Activate controller
ros2 control list_controllers
ros2 control switch_controllers --activate scaled_joint_trajectory_controller

# Terminal 1 Docker
ros2 run ur_client_library start_ursim.sh \
  -m ur3 \
  -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"

# Terminal 2-4 bringup.launch (Sim Default)
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.56.101 \
    planner_id:=RRTConnectkConfigDefault \
    ignore_if_busy:=true \
    trajectory_velocity_scaling:=0.1 \
    trajectory_acceleration_scaling:=0.1 \
    connection_type:=serial

# Terminal 2-4 bringup.launch (Real Default)
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.0.195 \
    planner_id:=RRTConnectkConfigDefault \
    ignore_if_busy:=true \
    trajectory_velocity_scaling:=0.1 \
    trajectory_acceleration_scaling:=0.1 \
    connection_type:=serial

# Terminal 2-4 bringup.launch (Sim star moveit config)
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.56.101 \
    planner_id:=RRTstarkConfigDefault \
    ignore_if_busy:=false

# gripper driver



# Terminal 2 Driver (Real Connection)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.0.195 launch_rviz:=false
# Terminal 2 Driver (Simulation without Gripper)
ros2 launch ur_robot_driver ur_control.launch.py \
    robot_ip:=192.168.56.101 \
    calibration_file:=~/git/RS2_SNL/ur_ws/src/ur3_planner/src/ur3_calibration.yaml \
    ur_type:=ur3 \
    launch_rviz:=false \
    trajectory_velocity_scaling:=0.3 \
    trajectory_acceleration_scaling:=0.3
# Terminal 2 Driver (Gripper)
ros2 launch ur_robot_driver ur_control.launch.py \
    robot_ip:=192.168.56.101 \
    calibration_file:=~/git/RS2_SNL/ur_ws/src/ur3_planner/src/ur3_calibration.yaml \
    ur_type:=ur3 \
    launch_rviz:=false \
    trajectory_velocity_scaling:=0.1 \
    trajectory_acceleration_scaling:=0.1 \
    use_fake_gripper:=true \
    gripper_connection_type:=tcp \
    headless_mode:=false

# External Connection
ros2 topic echo /io_and_status_controller/robot_program_running --once
ros2 topic echo /io_and_status_controller/robot_mode --once
# Controller
ros2 control list_controllers
ros2 control switch_controllers --activate scaled_joint_trajectory_controller

# Terminal 3 Moveit
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true

# Terminal 4 Listener RRTConnectkConfigDefault
ros2 run ur3_planner ur3_planner_listener \
    --ros-args -p planning_group:=ur_manipulator \
    -p execute_immediately:=true
# Terminal 4 Listener RRTstarkConfigDefault
ros2 run ur3_planner ur3_planner_listener \
    --ros-args -p planning_group:=ur_manipulator \
    -p execute_immediately:=true \
    -p planner_id:=RRTstarkConfigDefault

# Terminal 5 Goal Pose Publisher GUI
cd ~/git/RS2_SNL/ur_ws/
chmod +x src/ur3_planner/src/ur3_goal_gui.py
python3 src/ur3_planner/src/ur3_goal_gui.py

# Terminal 6 Gripper (Control Box)
ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 \
    connection_type:=tcp \
    use_fake_hardware:=true \
    ip_address:=192.168.1.1




# State tolerances failed for joint 5
Error: State tolerances failed for joint 5: Position Error: -6.283138, Position Tolerance: 0.200000
Cause: the last joint (wrist_3) is a continuous joint (no limits), and so the UR sometimes send -360 instead of 0 when it starts.
Fix: Use the Teach Pendant to set that joint to be 0 (in Move) before running External Control often fixes it

# Default Planner
ros2 run ur3_planner ur3_planner

# VScode include path fix
${workspaceFolder}/install/**
$
# Controllers
ros2 service list | grep controller_manager
ros2 control list_controllers
# Activate/Deactivate
cat $(ros2 pkg prefix ur_moveit_config)/share/ur_moveit_config/config/controllers.yaml
ros2 control switch_controllers --activate scaled_joint_trajectory_controller
ros2 control switch_controllers --activate joint_trajectory_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate scaled_joint_trajectory_controller

# External Control
ros2 topic echo /io_and_status_controller/robot_program_running --once
ros2 topic echo /io_and_status_controller/robot_mode --once

# Publish Goal Command
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],
    points: [{
      positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
      time_from_start: {sec: 2}
    }]
  }
}"

# Remove? root installation for ur_description
sudo mv /opt/ros/humble/share/ur_description.bak /opt/ros/humble/share/ur_description

# Ground Plane
ros2 param get /robot_state_publisher robot_description | grep ground_plane

# Installs
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Installs
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-visual-tools
sudo apt install libnet1-dev

# Installs
sudo apt install ros-humble-topic-tools

# ping
ping 192.168.56.101
ping 192.168.0.195
ipconfig

# ip
ip route | grep default | awk '{print $3}'
nc -zv 127.29.40.1 30001
#
nc -zv 127.0.0.1 30001   # primary port
nc -zv 127.0.0.1 29999   # dashboard port
# 
nc -zv 192.168.56.101 30001   # primary port
nc -zv 192.168.56.101 29999   # dashboard port
