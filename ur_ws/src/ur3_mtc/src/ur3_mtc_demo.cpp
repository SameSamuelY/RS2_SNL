#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3_mtc_demo");
namespace mtc = moveit::task_constructor;

class Ur3MTCNode
{
public:
  Ur3MTCNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("ur3_mtc_node", options) }
  {
    // Load kinematics parameters manually
    std::string kin_yaml;
    node_->declare_parameter("robot_description_kinematics", rclcpp::PARAMETER_STRING);
    if (node_->get_parameter("robot_description_kinematics", kin_yaml)) {
      // Already set via launch file, do nothing
    } else {
      RCLCPP_WARN(LOGGER, "robot_description_kinematics not set; planning may fail");
    }
  }

  void run()
  {
    task_ = createTask();
    try {
      task_.init();
    } catch (mtc::InitStageException& e) {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }
    if (!task_.plan(5)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Planning failed");
      return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      RCLCPP_ERROR_STREAM(LOGGER, "Execution failed");
    else
      RCLCPP_INFO(LOGGER, "Demo completed successfully");
  }

private:
  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("ur3_demo");
    task.loadRobotModel(node_);

    // Use your exact group names
    const std::string arm_group = "ur_manipulator";
    const std::string hand_group = "ur_onrobot_gripper";
    const std::string hand_frame = "gripper_tcp";  // tip link of gripper

    task.setProperty("group", arm_group);
    task.setProperty("eef", hand_group);
    task.setProperty("ik_frame", hand_frame);

    // Stage 1: Current state
    auto stage_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_current));

    // Stage 2: Open gripper (using joint interpolation planner)
    auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", planner);
    stage_open->setGroup(hand_group);
    stage_open->setGoal("open");  // assumes you have a group state named "open"
    task.add(std::move(stage_open));

    return task;
  }

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<Ur3MTCNode>(options);
  node->run();
  rclcpp::shutdown();
  return 0;
}