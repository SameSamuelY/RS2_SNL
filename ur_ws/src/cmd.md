cd ~/git/RS2_SNL/ur_ws/
# colcon build (all)
cd ~/git/RS2_SNL/ur_ws/
colcon build --symlink-install
    source install/setup.bash

# colcon build
cd ~/git/RS2_SNL/ur_ws/
colcon build --packages-select ur_description --symlink-install
colcon build --packages-select ur_robot_driver --symlink-install
colcon build --packages-select ur_moveit_config --symlink-install
colcon build --packages-select gui_control --symlink-install
colcon build --packages-select ur3_planner --symlink-install
colcon build --packages-select ur3_mtc --symlink-install
source install/setup.bash

# Source
source ~/git/RS2_SNL/ur_ws/install/setup.bash
source ~/git/RS2_SNL/src/install/setup.bash

# Docker 
http://localhost:6080/vnc.html

# Initialise docker (if not)
sudo service docker start

# Docker
ros2 run ur_client_library start_ursim.sh \
  -m ur3 \
  -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"

# bringup
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.56.101 \
    use_fake_gripper:=true \
    gripper_connection_type:=serial \
    rviz:=true

# Driver (standalone without bringup)
ros2 launch ur_robot_driver ur_control.launch.py \
    name:=ur_onrobot \
    robot_ip:=192.168.56.101 \
    ur_type:=ur3 \
    description_file:=ur_onrobot.urdf.xacro \
    moveit_config_file:=ur_onrobot.srdf.xacro \
    launch_rviz:=false \
    use_fake_gripper:=true \
    gripper_connection_type:=serial
# moveit (standalone without bringup)
ros2 launch ur_moveit_config ur_moveit.launch.py \
    name:=ur_onrobot \
    ur_type:=ur3 \
    launch_rviz:=true \
    description_file:=ur_onrobot.urdf.xacro \
    moveit_config_file:=ur_onrobot.srdf.xacro


# mtc listener
ros2 launch ur3_mtc mtc_pick_place_listener.launch.py \
    place_x:=-0.2 place_y:=0.2 place_z:=0.05

# mtc listener pub
ros2 topic pub --once /detection_result std_msgs/msg/String "{data: '{\"objects\": [{\"position\": {\"x\": 0.3, \"y\": 0.3, \"z\": 0.05}}]}'}"

ros2 topic pub --once /plan_goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.3, z: 0.07}, orientation: {w: 1.0}}}"

ros2 service call /trigger_pick_and_place std_srvs/srv/Trigger '{}'

# install 
sudo apt install nlohmann-json3-dev

# error log stream
sudo apt install ros-humble-rqt-console
ros2 run rqt_console rqt_console

# gripper & controllers
ros2 control list_controllers
ros2 control switch_controllers --activate scaled_joint_trajectory_controller
ros2 run controller_manager spawner finger_width_trajectory_controller
ros2 topic pub --once /ur3_gripper_cmd std_msgs/msg/Float64MultiArray "{data: [0.11]}"

# Network
sudo ip route del default via 192.168.0.1
sudo ip route del default via 192.168.1.1
sudo sh -c 'echo "nameserver 8.8.8.8" >> /etc/resolv.conf'

# mtc k
ros2 launch ur3_mtc mtc_pick_place_kinematics.launch.py

ros2 launch ur3_mtc mtc_pick_place_kinematics.launch.py \
    pick_x:=-0.1 pick_y:=-0.1 pick_z:=0.05 \
    place_x:=0.1 place_y:=0.1 place_z:=0.05 \
    place_qx:=0.0 place_qy:=0.0 place_qz:=0.0 place_qw:=1.0

ros2 launch ur3_mtc mtc_pick_place_kinematics.launch.py \
    pick_x:=-0.2 pick_y:=-0.2 pick_z:=0.05 \
    place_x:=0.2 place_y:=0.2 place_z:=0.05 \
    place_qx:=0.0 place_qy:=0.0 place_qz:=0.0 place_qw:=1.0

#
#
#

# Terminal 1 Docker
ros2 run ur_client_library start_ursim.sh \
  -m ur3 \
  -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"

# Terminal 2-4 bringup.launch (Sim Default)
ros2 launch ur3_planner bringup.launch.py \
    robot_ip:=192.168.56.101 \
    gripper_connection_type:=serial \
    ur_type:=ur3 \
    name:=ur_onrobot \
    description_file:=ur.urdf.xacro \
    moveit_config_file:=ur.srdf.xacro


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


#
#
#

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

# Installs mtc
cd ~/git/RS2_SNL/ur_ws/src/moveit2_tutorials_ur_onrobot/ur_onrobot_mtc
rosdep install --from-paths src --ignore-src -r -y

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
