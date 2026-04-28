#include <memory>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3_mtc_simple");
namespace mtc = moveit::task_constructor;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create node with option to automatically declare parameters from overrides
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ur3_mtc_simple", node_options);

  // Declare the parameter explicitly
  node->declare_parameter<std::string>("robot_description_kinematics", "");

  // Now load kinematics.yaml and set it
  std::string kin_path = "/home/samuel/git/RS2_SNL/ur_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml";
  std::ifstream file(kin_path);
  if (file.is_open()) {
    std::stringstream buffer;
    buffer << file.rdbuf();
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics", buffer.str()));
    RCLCPP_INFO(LOGGER, "Loaded kinematics.yaml");
  } else {
    RCLCPP_WARN(LOGGER, "kinematics.yaml not found");
  }

  mtc::Task task;
  task.stages()->setName("simple_move");
  task.loadRobotModel(node);

  const std::string arm_group = "ur_manipulator";
  const std::string eef = "gripper_tcp";
  task.setProperty("group", arm_group);
  task.setProperty("ik_frame", eef);

  // Stage 1: current robot state
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  // Stage 2: Cartesian planner
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);

  // Stage 3: move to target pose
  auto stage_move = std::make_unique<mtc::stages::MoveTo>("move_to_target", cartesian_planner);
  stage_move->setGroup(arm_group);
  stage_move->setProperty("ik_frame", eef);

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "base_link";
  target.pose.position.x = 0.3;
  target.pose.position.y = -0.2;
  target.pose.position.z = 0.5;
  target.pose.orientation.w = 1.0;
  stage_move->setGoal(target);
  task.add(std::move(stage_move));

  try {
    task.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return 1;
  }

  if (!task.plan(5)) {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return 1;
  }

  task.introspection().publishSolution(*task.solutions().front());
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution failed");
    return 1;
  }

  RCLCPP_INFO(LOGGER, "Motion completed successfully");
  rclcpp::shutdown();
  return 0;
}