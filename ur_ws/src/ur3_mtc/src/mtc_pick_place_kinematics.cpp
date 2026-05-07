#include <memory>
#include <thread>
#include <chrono>
#include <map>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

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
  MTCPickPlaceKinematicsNode(const rclcpp::NodeOptions &options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  double object_radius_;
  double object_width_;

  double pick_x_, pick_y_, pick_z_;
  double place_x_, place_y_, place_z_;
  double place_qx_, place_qy_, place_qz_, place_qw_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceKinematicsNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCPickPlaceKinematicsNode::MTCPickPlaceKinematicsNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_pick_place_kinematics", options)}
{
  node_->get_parameter_or<double>("pick_x", pick_x_, -0.3);
  node_->get_parameter_or<double>("pick_y", pick_y_, -0.3);
  node_->get_parameter_or<double>("pick_z", pick_z_, 0.05);
  node_->get_parameter_or<double>("place_x", place_x_, 0.3);
  node_->get_parameter_or<double>("place_y", place_y_, 0.3);
  node_->get_parameter_or<double>("place_z", place_z_, 0.05);
  node_->get_parameter_or<double>("place_qx", place_qx_, 0.0);
  node_->get_parameter_or<double>("place_qy", place_qy_, 0.0);
  node_->get_parameter_or<double>("place_qz", place_qz_, 0.0);
  node_->get_parameter_or<double>("place_qw", place_qw_, 0.0);

  RCLCPP_INFO(LOGGER, "Pick  position: (%.2f, %.2f, %.2f)", pick_x_, pick_y_, pick_z_);
  RCLCPP_INFO(LOGGER, "Place position: (%.2f, %.2f, %.2f)", place_x_, place_y_, place_z_);
  RCLCPP_INFO(LOGGER, "Place orientation: (%.2f, %.2f, %.2f, %.2f)", place_qx_, place_qy_, place_qz_, place_qw_);
}

void MTCPickPlaceKinematicsNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.05, 0.025}; // height 0.05 m, radius 0.025 m

  object_radius_ = object.primitives[0].dimensions[1];
  object_width_ = 2 * object_radius_ + 0.02;

  geometry_msgs::msg::Pose pose;
  pose.position.x = pick_x_;
  pose.position.y = pick_y_;
  pose.position.z = pick_z_;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCPickPlaceKinematicsNode::doTask()
{
  task_ = createTask();
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Caught InitStageException during task initialization: " << e.what());
    return;
  }
  catch (std::exception &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Caught standard exception during task initialization: " << e.what());
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed, no solutions found");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  try
  {
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
      return;
    }
  }
  catch (std::exception &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Caught exception during task execution: " << e.what());
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
  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  // Stage 1: current state
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  joint_planner->setMaxVelocityScalingFactor(0.1);
  joint_planner->setMaxAccelerationScalingFactor(0.1);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(.001);

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");
  sampling_planner->setMaxVelocityScalingFactor(0.1);
  sampling_planner->setMaxAccelerationScalingFactor(0.1);

  // Helper for arm stages
  auto add_arm_stage = [&](const std::string &name, const std::map<std::string, double> &joints)
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
      {"wrist_3_joint", 0.0}};

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
  pre_grasp_joints["elbow_joint"] = -1.57; // lift up before moving over to place side

  // Place pose (lower to drop)
  std::map<std::string, double> place_joints = grasp_joints;
  place_joints["shoulder_pan_joint"] = 1.0;

  // Place pre‑pose (opposite side)
  std::map<std::string, double> place_pre_joints = place_joints;
  place_pre_joints["elbow_joint"] = -1.57; // lift up before moving over to place side

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
      mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
  stage_move_to_pick->setTimeout(15.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage *attach_object_stage =
      nullptr; // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "group", "ik_frame"});

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.01, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      // vec.header.frame_id = hand_frame;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
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
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q_x = 
      (
        // Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())
      );
      Eigen::Quaterniond q_y = 
      (
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        // Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())
      );
      grasp_frame_transform.linear() = q_x.matrix();
      RCLCPP_INFO(LOGGER, "q_x: (%.2f, %.2f, %.2f, %.2f)", q_x.x(), q_x.y(), q_x.z(), q_x.w());
      RCLCPP_INFO(LOGGER, "q_y: (%.2f, %.2f, %.2f, %.2f)", q_y.x(), q_y.y(), q_y.z(), q_y.w());


      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(3);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
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
      stage->setGoal({{"finger_width", object_width_}});
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
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
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

  // // Stage 6: attach object to robot in planning scene
  // auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  // attach->attachObject("object", hand_frame);
  // task.add(std::move(attach));

  // // Stage 7: move to place pose
  // add_arm_stage("lift", pre_grasp_joints);
  // add_arm_stage("place pre", place_pre_joints); // move to place side
  // add_arm_stage("place", place_joints);

  // // Stage 8: open gripper to release object
  // auto open_place = std::make_unique<mtc::stages::MoveTo>("open gripper place", joint_planner);
  // open_place->setGroup(hand_group);
  // open_place->setGoal("open");
  // task.add(std::move(open_place));

  // // Stage 9: detach object from robot in planning scene
  // auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  // detach->detachObject("object", hand_frame);
  // task.add(std::move(detach));

  // add_arm_stage("retreat", home_joints); // retreat

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group, sampling_planner}});
    stage_move_to_place->setTimeout(15.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "group", "ik_frame"});

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = place_x_;
      target_pose_msg.pose.position.y = place_y_;
      target_pose_msg.pose.position.z = place_z_;
      target_pose_msg.pose.orientation.x = place_qx_;
      target_pose_msg.pose.orientation.y = place_qy_;
      target_pose_msg.pose.orientation.z = place_qz_;
      target_pose_msg.pose.orientation.w = place_qw_;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", joint_planner);
      stage->setGroup(hand_group);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      // auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      // stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      // stage->setMinMaxDistance(0.05, 0.2);
      // stage->setIKFrame(hand_frame);
      // stage->properties().set("marker_ns", "retreat");

      // // Set retreat direction
      // geometry_msgs::msg::Vector3Stamped vec;
      // vec.header.frame_id = "world";
      // vec.vector.z = 1.0;
      // stage->setDirection(vec);
      // place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    // auto stage = std::make_unique<mtc::stages::MoveTo>("return home", joint_planner);
    // stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    // stage->setGoal("up");
    // stage->setTimeout(15.0);
    // task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_node = std::make_shared<MTCPickPlaceKinematicsNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]()
                                                   {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface()); });

  mtc_node->setupPlanningScene();
  mtc_node->doTask();

  rclcpp::shutdown();
  spin_thread->join();
  return 0;
}