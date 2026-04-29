#include <memory>
#include <thread>
#include <chrono>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place");
namespace mtc = moveit::task_constructor;

class MTCPickPlaceNode
{
public:
  MTCPickPlaceNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCPickPlaceNode::MTCPickPlaceNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place", options) }
{
}

void MTCPickPlaceNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "cylinder_object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.03};  // height 0.1 m, radius 0.03 m

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;        // moved further away from robot base
  pose.position.y = -0.2;
  pose.position.z = 0.06;
  pose.orientation.y = 0.707;
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCPickPlaceNode::doTask()
{
  task_ = createTask();
  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }
  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Pick and place completed successfully");
}

mtc::Task MTCPickPlaceNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("pick_and_place");
  task.loadRobotModel(node_);

  const std::string arm_group = "ur_manipulator";
  const std::string hand_group = "ur_onrobot_gripper";
  const std::string hand_frame = "gripper_tcp";

  task.setProperty("group", arm_group);
  task.setProperty("eef", hand_group);
  task.setProperty("ik_frame", hand_frame);

  auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // Stage 1: current state
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  // Stage 2: open gripper
  auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
  open->setGroup(hand_group);
  open->setGoal("open");
  task.add(std::move(open));

  // Helper for arm stages
  auto add_arm_stage = [&](const std::string& name, const std::map<std::string, double>& joints) {
    auto stage = std::make_unique<mtc::stages::MoveTo>(name, joint_planner);
    stage->setGroup(arm_group);
    stage->setGoal(joints);
    task.add(std::move(stage));
  };

  // Home configuration (up, away from object)
  std::map<std::string, double> home_joints = {
    {"shoulder_pan_joint", 0.0},
    {"shoulder_lift_joint", -1.57},
    {"elbow_joint", 0.0},
    {"wrist_1_joint", -1.57},
    {"wrist_2_joint", 0.0},
    {"wrist_3_joint", 0.0}
  };

  // Pre-grasp (above object) – same as home, then we will move to a grasp pose later
  std::map<std::string, double> pre_grasp_joints = home_joints;

  // Grasp configuration (lower elbow and adjust wrist to reach object at x=0.7)
  // These are approximate; you will need to tune them using RViz.
  std::map<std::string, double> grasp_joints = home_joints;
  grasp_joints["shoulder_pan_joint"] = -3.30;
  grasp_joints["shoulder_lift_joint"] = -1.56;
  grasp_joints["elbow_joint"] = -1.93;
  grasp_joints["wrist_1_joint"] = -1.14;
  grasp_joints["wrist_2_joint"] = 1.56;
  grasp_joints["wrist_3_joint"] = 1.38;

  // Place pre‑pose (opposite side)
  std::map<std::string, double> place_pre_joints = home_joints;
  place_pre_joints["shoulder_pan_joint"] = -0.3;
  place_pre_joints["shoulder_lift_joint"] = -1.2;
  place_pre_joints["elbow_joint"] = 0.5;
  place_pre_joints["wrist_1_joint"] = -1.0;

  // Place pose (lower to drop)
  std::map<std::string, double> place_joints = place_pre_joints;
  place_joints["elbow_joint"] = 0.9;

  // Stages
  add_arm_stage("pre grasp", pre_grasp_joints);   // safe above object
  add_arm_stage("grasp", grasp_joints);           // move to object
  auto close = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
  close->setGroup(hand_group);
  close->setGoal("closed");
  task.add(std::move(close));
  auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  attach->attachObject("cylinder_object", hand_frame);
  task.add(std::move(attach));
  add_arm_stage("lift", pre_grasp_joints);        // lift object
  add_arm_stage("place pre", place_pre_joints);   // move to place side
  add_arm_stage("place", place_joints);           // lower
  auto open_place = std::make_unique<mtc::stages::MoveTo>("open gripper place", joint_planner);
  open_place->setGroup(hand_group);
  open_place->setGoal("open");
  task.add(std::move(open_place));
  auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  detach->detachObject("cylinder_object", hand_frame);
  task.add(std::move(detach));
  add_arm_stage("retreat", home_joints);     // retreat

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_node = std::make_shared<MTCPickPlaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });

  mtc_node->setupPlanningScene();
  mtc_node->doTask();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}