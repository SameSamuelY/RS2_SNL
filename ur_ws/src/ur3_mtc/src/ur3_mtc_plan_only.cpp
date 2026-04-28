#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur3_mtc_plan_only");
namespace mtc = moveit::task_constructor;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ur3_mtc_plan_only");

  mtc::Task task;
  task.stages()->setName("joint_demo");
  task.loadRobotModel(node);

  const std::string arm_group = "ur_manipulator";
  task.setProperty("group", arm_group);

  // Stage 1: current state
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  // Stage 2: joint interpolation planner
  auto planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // Stage 3: move to joint target
  auto stage_move = std::make_unique<mtc::stages::MoveTo>("move_to_target", planner);
  stage_move->setGroup(arm_group);

  std::map<std::string, double> joint_goal;
  joint_goal["shoulder_pan_joint"] = 0.2;
  joint_goal["shoulder_lift_joint"] = -0.8;
  joint_goal["elbow_joint"] = 0.5;
  joint_goal["wrist_1_joint"] = -0.3;
  joint_goal["wrist_2_joint"] = 0.2;
  joint_goal["wrist_3_joint"] = 0.0;
  stage_move->setGoal(joint_goal);
  task.add(std::move(stage_move));

  try {
    task.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return 1;
  }

  if (task.plan(5)) {
    task.introspection().publishSolution(*task.solutions().front());
    RCLCPP_INFO(LOGGER, "Planning succeeded – solution published in RViz (execution skipped)");
  } else {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}