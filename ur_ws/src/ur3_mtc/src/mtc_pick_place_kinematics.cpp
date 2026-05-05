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

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_kinematics");
namespace mtc = moveit::task_constructor;

class MTCPickPlaceKinematicsNode
{
public:
  MTCPickPlaceKinematicsNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceKinematicsNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCPickPlaceKinematicsNode::MTCPickPlaceKinematicsNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place_kinematics", options) }
{
}

void MTCPickPlaceKinematicsNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.03};  // height 0.1 m, radius 0.03 m

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.3;        // moved further away from robot base
  pose.position.y = -0.162;
  pose.position.z = 0.03;       // on the table
  pose.orientation.x = 0;
  pose.orientation.y = 1;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCPickPlaceKinematicsNode::doTask()
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

mtc::Task MTCPickPlaceKinematicsNode::createTask()
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

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  #pragma GCC diagnostic pop

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(.01);

  // Helper for arm stages
  auto add_arm_stage = [&](const std::string& name, const std::map<std::string, double>& joints) 
  {
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

  // Grasp configuration (lower elbow and adjust wrist to reach object)
  // These are approximate; you will need to tune them using RViz.
  std::map<std::string, double> grasp_joints = home_joints;
    grasp_joints["shoulder_pan_joint"] = -3.30;
    grasp_joints["shoulder_lift_joint"] = -1.56;
    grasp_joints["elbow_joint"] = -1.93;
    grasp_joints["wrist_1_joint"] = -1.14;
    grasp_joints["wrist_2_joint"] = 1.56;
    grasp_joints["wrist_3_joint"] = 1.38;

  // Pre-grasp (above object) – same as home, then we will move to a grasp pose later
  std::map<std::string, double> pre_grasp_joints = grasp_joints;
    pre_grasp_joints["elbow_joint"] = -1.57;  // lift up before moving over to place side

  // Place pose (lower to drop)
  std::map<std::string, double> place_joints = grasp_joints;
    place_joints["shoulder_pan_joint"] = 1.0;

  // Place pre‑pose (opposite side)
  std::map<std::string, double> place_pre_joints = place_joints;
    place_pre_joints["elbow_joint"] = -1.57;  // lift up before moving over to place side
  

  // Stage 1: current state
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // Stage 2: open gripper
  auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
  open->setGroup(hand_group);
  open->setGoal("open");
  task.add(std::move(open));

  // // Stage 3: move above object
  // add_arm_stage("pre grasp", pre_grasp_joints);   // safe above object
  // // Stage 4: move to grasp pose
  // add_arm_stage("grasp", grasp_joints);           // move to object

  // // Stage 5: close gripper
  // auto close = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_planner);
  // close->setGroup(hand_group);
  // close->setGoal({{"finger_width", 0.062}});
  // task.add(std::move(close));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group, sampling_planner } });
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 36);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
                      
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      
        // Compute IK
        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(3);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", joint_planner);
      stage->setGroup(hand_group);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.20);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // Stage 6: attach object to robot in planning scene
  auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  attach->attachObject("object", hand_frame);
  task.add(std::move(attach));

  // Stage 7: move to place pose
  add_arm_stage("lift", pre_grasp_joints); 
  add_arm_stage("place pre", place_pre_joints);   // move to place side
  add_arm_stage("place", place_joints); 
  
  // Stage 8: open gripper to release object
  auto open_place = std::make_unique<mtc::stages::MoveTo>("open gripper place", joint_planner);
  open_place->setGroup(hand_group);
  open_place->setGoal("open");
  task.add(std::move(open_place));

  // Stage 9: detach object from robot in planning scene
  auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  detach->detachObject("object", hand_frame);
  task.add(std::move(detach));

  add_arm_stage("retreat", home_joints);     // retreat

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_node = std::make_shared<MTCPickPlaceKinematicsNode>(options);
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