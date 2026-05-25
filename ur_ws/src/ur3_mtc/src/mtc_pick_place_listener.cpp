#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <map>
#include <cmath>
#include <string>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_listener");
namespace mtc = moveit::task_constructor;

class MTCPickPlaceListener : public rclcpp::Node
{
public:
    MTCPickPlaceListener(const rclcpp::NodeOptions &options);
    ~MTCPickPlaceListener();
    void addObjectToScene(double x, double y, double z);
    void returnHome();
    void runPickAndPlace();

private:
    mtc::Task createTask();
    void detectionCallback(const std_msgs::msg::String::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void triggerCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    bool activateController(const std::string& controller_name);
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr 
        controller_switch_cli_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_sub_;
    std::mutex pick_pose_mutex_;
    bool pick_pose_received_ = false;
    double received_pick_x_, received_pick_y_, received_pick_z_;

    double object_radius_;
    double object_width_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_pose_sub_;
    std::mutex place_pose_mutex_;
    bool place_pose_received_ = false;
    geometry_msgs::msg::PoseStamped received_place_pose_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
    std::atomic<bool> is_busy_{false};
    std::thread planner_thread_;
    const int max_attempts_ = 2; // first attempt + one retry

    struct Obstacle
    {
        double x, y, z;
        double radius;     // for cylinder (or use box dimensions)
        std::string shape; // "cylinder", "box"
    };
    std::vector<Obstacle> obstacles_;
    std::mutex obstacles_mutex_;

    double pick_x_, pick_y_, pick_z_;
    double place_x_, place_y_, place_z_;
    double place_qx_, place_qy_, place_qz_, place_qw_;
};

MTCPickPlaceListener::MTCPickPlaceListener(const rclcpp::NodeOptions &options)
    : Node("mtc_pick_place_listener", options)
{
    // Declare and get place position/orientation parameters
    this->get_parameter_or<double>("place_x", place_x_, 0.3);
    this->get_parameter_or<double>("place_y", place_y_, 0.3);
    this->get_parameter_or<double>("place_z", place_z_, 0.05);
    this->get_parameter_or<double>("place_qx", place_qx_, 0.0);
    this->get_parameter_or<double>("place_qy", place_qy_, 0.0);
    this->get_parameter_or<double>("place_qz", place_qz_, 0.0);
    this->get_parameter_or<double>("place_qw", place_qw_, 0.0);

    // Subscribe to detection topic
    detection_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/detection_result", 10,
        std::bind(&MTCPickPlaceListener::detectionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MTC Pick and Place Listener started. Waiting for detection...");

    place_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/plan_goal_pose", 10,
        std::bind(&MTCPickPlaceListener::goalCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Listening for place goals on /plan_goal_pose");

    trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/trigger_pick_and_place",
        std::bind(&MTCPickPlaceListener::triggerCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service 'trigger_pick_and_place' ready. Call to start pick-and-place.");
}

MTCPickPlaceListener::~MTCPickPlaceListener()
{
    if (planner_thread_.joinable())
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for pick-and-place thread to finish...");
        planner_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Thread joined.");
    }
}

void MTCPickPlaceListener::detectionCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pick_pose_mutex_);
    RCLCPP_INFO(this->get_logger(), "Received detection: %s", msg->data.c_str());

    try
    {
        auto json = nlohmann::json::parse(msg->data);
        // Expecting format: { "objects": [ { "position": { "x": 0.1, "y": 0.2, "z": 0.05 } } ] }
        if (!json.contains("objects") || json["objects"].empty())
        {
            RCLCPP_WARN(this->get_logger(), "No 'objects' array or empty in detection message");
            return;
        }
        auto &obj = json["objects"][0];
        if (!obj.contains("position"))
        {
            RCLCPP_WARN(this->get_logger(), "Object missing 'position' field");
            return;
        }
        received_pick_x_ = obj["position"]["x"];
        received_pick_y_ = obj["position"]["y"];
        received_pick_z_ = obj["position"]["z"];
        pick_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Pick position: (%.3f, %.3f, %.3f). Waiting for trigger.",
                    received_pick_x_, received_pick_y_, received_pick_z_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse detection JSON: %s", e.what());
        return;
    }
}

void MTCPickPlaceListener::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(place_pose_mutex_);
    received_place_pose_ = *msg;
    place_pose_received_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Received new place goal: (%.3f, %.3f, %.3f) orientation: (%.3f, %.3f, %.3f, %.3f)",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void MTCPickPlaceListener::triggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req; // unused

    // Prevent concurrent executions
    if (is_busy_.exchange(true))
    {
        res->success = false;
        res->message = "Already executing pick-and-place. Try later.";
        return;
    }

    // Check pick pose
    {
        std::lock_guard<std::mutex> lock(pick_pose_mutex_);
        if (!pick_pose_received_)
        {
            res->success = false;
            res->message = "No detection received yet. Please publish a detection first.";
            RCLCPP_WARN(this->get_logger(), "Trigger called but no detection pose available.");
            is_busy_ = false;
            return;
        }
        // Copy received pick coordinates to member variables used by createTask()
        pick_x_ = received_pick_x_;
        pick_y_ = received_pick_y_;
        pick_z_ = received_pick_z_;
        pick_pose_received_ = false;
    }

    // Check place pose
    {
        std::lock_guard<std::mutex> lock(place_pose_mutex_);
        if (place_pose_received_)
        {
            place_x_ = received_place_pose_.pose.position.x;
            place_y_ = received_place_pose_.pose.position.y;
            place_z_ = received_place_pose_.pose.position.z;
            place_qx_ = received_place_pose_.pose.orientation.x;
            place_qy_ = received_place_pose_.pose.orientation.y;
            place_qz_ = received_place_pose_.pose.orientation.z;
            place_qw_ = received_place_pose_.pose.orientation.w;
            place_pose_received_ = false;
            RCLCPP_INFO(this->get_logger(), "Using received place pose from /plan_goal_pose");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No place pose received, using default parameters");
        }
    }

    if (planner_thread_.joinable())
    {
        planner_thread_.join();
    }
    // Launch new thread and store it (not detached)
    planner_thread_ = std::thread([this]()
                                  {
        runPickAndPlace();
        is_busy_ = false; });

    res->success = true;
    res->message = "Starting pick-and-place asynchronously.";
    RCLCPP_INFO(this->get_logger(), "Trigger accepted. Running pick-and-place asynchronously.");
}

bool MTCPickPlaceListener::activateController(const std::string& controller_name)
{
    controller_switch_cli_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    if (!controller_switch_cli_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Switch controller service not available");
        return false;
    }

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = {controller_name};
    auto future = controller_switch_cli_->async_send_request(req);
    if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
    {
        RCLCPP_INFO(this->get_logger(), "Activated controller '%s' (or already active)", controller_name.c_str());
        return true;
    }
    else 
    {
        RCLCPP_WARN(this->get_logger(), "Activation service call for '%s' timed out; continuing anyway", controller_name.c_str());
        return false;
    }
}

void MTCPickPlaceListener::addObjectToScene(double x, double y, double z)
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.05, 0.025}; // height 0.05 m, radius 0.025 m

    object_radius_ = object.primitives[0].dimensions[1];
    object_width_ = 2 * object_radius_;
    // object_width_ = 0.05;

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    object.primitive_poses.push_back(pose);
    object.operation = object.ADD;

    moveit_msgs::msg::CollisionObject obstacle1;
    obstacle1.id = "obstacle1";
    obstacle1.header.frame_id = "world";
    obstacle1.primitives.resize(1);
    obstacle1.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obstacle1.primitives[0].dimensions = {0.05, 0.05, 0.05};
    geometry_msgs::msg::Pose obs_pose1;
    obs_pose1.position.x = x - 0.07;
    obs_pose1.position.y = y;
    obs_pose1.position.z = z;
    obs_pose1.orientation.w = 1.0;
    obstacle1.primitive_poses.push_back(obs_pose1);
    obstacle1.operation = obstacle1.ADD;

    moveit_msgs::msg::CollisionObject obstacle2;
    obstacle2.id = "obstacle2";
    obstacle2.header.frame_id = "world";
    obstacle2.primitives.resize(1);
    obstacle2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obstacle2.primitives[0].dimensions = {0.05, 0.05, 0.05};
    geometry_msgs::msg::Pose obs_pose2;
    obs_pose2.position.x = x;
    obs_pose2.position.y = y + 0.07;
    obs_pose2.position.z = z;
    obs_pose2.orientation.w = 1.0;
    obstacle2.primitive_poses.push_back(obs_pose2);
    obstacle2.operation = obstacle2.ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
    psi.applyCollisionObject(obstacle1);
    psi.applyCollisionObject(obstacle2);
    RCLCPP_INFO(this->get_logger(), "Waiting for object to appear in planning scene...");
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(1))
    {
        auto objects = psi.getObjects();
        if (objects.find("object") != objects.end())
        {
            RCLCPP_INFO(this->get_logger(), "Object confirmed in scene at (%.3f, %.3f, %.3f)", x, y, z);
            return;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_WARN(this->get_logger(), "Object not seen after 1 second - proceeding anyway.");
}

void MTCPickPlaceListener::returnHome()
{
    RCLCPP_INFO(this->get_logger(), "Moving to safe home pose...");
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_manipulator");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    move_group->setGoalJointTolerance(0.1);       // rad
    move_group->setGoalPositionTolerance(0.05);   // m
    move_group->setGoalOrientationTolerance(0.1); // rad
    move_group->setPlannerId("RRTConnectkConfigDefault");
    move_group->setPlanningTime(30.0);
    move_group->setMaxVelocityScalingFactor(0.3);
    move_group->setMaxAccelerationScalingFactor(0.3);
    move_group->setNamedTarget("up");

    auto success = static_cast<bool>(move_group->move());
    if (!success)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to move home, may already be at safe pose.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Reached safe home pose.");
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
}

void MTCPickPlaceListener::runPickAndPlace()
{
    if (!activateController("scaled_joint_trajectory_controller")) {
        RCLCPP_ERROR(this->get_logger(), "Cannot proceed without active UR3 trajectory controller");
        return;
    }
    if (!activateController("finger_width_trajectory_controller")) {
        RCLCPP_ERROR(this->get_logger(), "Cannot proceed without active gripper controller");
        return;
    }

    for (int attempt = 1; attempt <= max_attempts_; ++attempt)
    {
        RCLCPP_INFO(this->get_logger(), "Pick-and-place attempt %d/%d", attempt, max_attempts_);
        addObjectToScene(pick_x_, pick_y_, pick_z_);

        mtc::Task task = createTask();
        try
        {
            task.init();
        }
        catch (mtc::InitStageException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Caught InitStageException during task initialization: " << e.what());
            if (attempt < max_attempts_)
            {
                RCLCPP_INFO(this->get_logger(), "Retrying pick-and-place after initialization failure...");
                returnHome();
                continue; // retry
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Max attempts reached. Aborting pick-and-place.");
                return;
            }
            return;
        }

        if (!task.plan(5))
        {
            RCLCPP_ERROR(this->get_logger(), "Task planning failed on attempt %d", attempt);
            if (attempt < max_attempts_)
            {
                RCLCPP_INFO(this->get_logger(), "Retrying pick-and-place after planning failure...");
                returnHome();
                continue; // retry
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Max attempts reached. Aborting pick-and-place.");
            }
            return;
        }

        task.introspection().publishSolution(*task.solutions().front());
        try
        {
            auto result = task.execute(*task.solutions().front());
            if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "Task execution failed on attempt " << attempt << " with error code: " << result.val);
                if (attempt < max_attempts_)
                {
                    RCLCPP_INFO(this->get_logger(), "Retrying pick-and-place after execution failure...");
                    returnHome();
                    continue; // retry
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Max attempts reached. Aborting pick-and-place.");
                }
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Pick-and-place executed successfully on attempt %d", attempt);
                break; // exit loop on success
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Caught exception during task execution: " << e.what());
            if (attempt < max_attempts_)
            {
                RCLCPP_INFO(this->get_logger(), "Retrying pick-and-place after execution exception...");
                returnHome();
                continue; // retry
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Max attempts reached. Aborting pick-and-place.");
            }
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Pick and place completed successfully");
    }
}

mtc::Task MTCPickPlaceListener::createTask()
{
    // This function is identical to MTCPickPlaceKinematicsNode::createTask(),
    // but uses member variables for pick_* and place_*.
    // node_ => shared_from_this()
    // object_width_
    mtc::Task task;
    task.stages()->setName("pick_and_place");
    task.loadRobotModel(shared_from_this()); // use this node

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
    cartesian_planner->setStepSize(.05);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    sampling_planner->setPlannerId("RRTConnectkConfigDefault");
    sampling_planner->setMaxVelocityScalingFactor(0.1);
    sampling_planner->setMaxAccelerationScalingFactor(0.1);

    // Stage 2: open gripper
    auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joint_planner);
    open->setGroup(hand_group);
    open->setGoal("open");
    task.add(std::move(open));

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
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
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
            auto grasp_fallback = std::make_unique<mtc::Fallbacks>("grasp strategies");
            task.properties().exposeTo(
                grasp_fallback->properties(),
                {"eef", "group", "ik_frame"});
            grasp_fallback->properties().configureInitFrom(
                mtc::Stage::PARENT,
                {"eef", "group", "ik_frame"});

            {
                auto grasp_top = std::make_unique<mtc::SerialContainer>("Top grasp");
                task.properties().exposeTo(
                    grasp_top->properties(),
                    {"eef", "group", "ik_frame"});
                grasp_top->properties().configureInitFrom(
                    mtc::Stage::PARENT,
                    {"eef", "group", "ik_frame"});

                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate top grasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose("open");
                stage->setObject("object");
                stage->setAngleDelta(M_PI / 36);
                stage->setMonitoredStage(current_state_ptr);

                Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
                Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
                grasp_frame_transform.linear() = q.matrix();

                // Compute IK
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("Top grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(5);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(grasp_frame_transform, hand_frame);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

                grasp_top->insert(std::move(wrapper));
                grasp_fallback->add(std::move(grasp_top));
            }

            {
                auto grasp_side = std::make_unique<mtc::SerialContainer>("Side grasp");
                task.properties().exposeTo(
                    grasp_side->properties(),
                    {"eef", "group", "ik_frame"});
                grasp_side->properties().configureInitFrom(
                    mtc::Stage::PARENT,
                    {"eef", "group", "ik_frame"});

                auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate side grasp pose");
                stage->properties().configureInitFrom(mtc::Stage::PARENT);
                stage->properties().set("marker_ns", "grasp_pose");
                stage->setPreGraspPose("open");
                stage->setObject("object");
                stage->setAngleDelta(M_PI / 18);
                stage->setMonitoredStage(current_state_ptr);

                Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
                Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI * 0.75, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
                grasp_frame_transform.linear() = q.matrix();

                // Compute IK
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>("Side grasp pose IK", std::move(stage));
                wrapper->setMaxIKSolutions(10);
                wrapper->setMinSolutionDistance(1.0);
                wrapper->setIKFrame(grasp_frame_transform, hand_frame);
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
                wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

                grasp_side->insert(std::move(wrapper));
                grasp_fallback->add(std::move(grasp_side));
            }
            grasp->insert(std::move(grasp_fallback));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
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
            wrapper->setMaxIKSolutions(20);
            wrapper->setMinSolutionDistance(0.5);
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

        // Retreat stage commented out

        task.add(std::move(place));
    }

    return task;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto mtc_node = std::make_shared<MTCPickPlaceListener>(options);

    rclcpp::spin(mtc_node);

    rclcpp::shutdown();
    return 0;
}