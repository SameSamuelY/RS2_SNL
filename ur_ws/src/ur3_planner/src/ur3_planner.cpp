#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur3_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("ur3_planner");

  // Create MoveGroupInterface for the UR3 arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set a target pose (adjust coordinates to your workspace)
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.2;
    msg.position.y = 0.3;
    msg.position.z = 0.4;
    msg.orientation.x = 0.0;
    msg.orientation.y = 1.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const success = static_cast<bool>(move_group_interface.plan(plan));

  // Execute if successful
  if(success) {
    RCLCPP_INFO(logger, "Planning succeeded, executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}