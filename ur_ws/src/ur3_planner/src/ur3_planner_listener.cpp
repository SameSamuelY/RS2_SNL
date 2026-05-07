#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>


class Ur3PlannerListener : public rclcpp::Node
{
public:
  Ur3PlannerListener() : Node("ur3_planner_listener"), is_busy_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    this->declare_parameter<bool>("execute_immediately", true);
    this->declare_parameter<bool>("ignore_if_busy", true);
    this->declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

    this->get_parameter("planning_group", planning_group_);
    this->get_parameter("execute_immediately", execute_immediately_);
    this->get_parameter("ignore_if_busy", ignore_if_busy_);
    this->get_parameter("planner_id", planner_id_);

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/ur3_goal_pose", 10,
      std::bind(&Ur3PlannerListener::poseCallback, this, std::placeholders::_1)
    );

    ex_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/io_and_status_controller/robot_program_running", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
          {
              std::lock_guard<std::mutex> lock(ex_control_mutex_);
              ex_control_running_ = msg->data;
          }
          ex_control_condition_.notify_all();
      });

    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/motion_status", 10);

    switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");

    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/finger_width_trajectory_controller/commands", 10);
    gripper_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/ur3_gripper_cmd", 10,
        std::bind(&Ur3PlannerListener::gripperCallback, this, std::placeholders::_1));

    publish_status("idle");
    RCLCPP_INFO(this->get_logger(), "Node started. Listening for poses on /ur3_goal_pose");
    RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());
  }

private:
  std::string planning_group_;
  bool execute_immediately_;
  bool ignore_if_busy_;
  std::string planner_id_;

  bool wait_for_ex_control_running(std::chrono::seconds timeout = std::chrono::seconds(5)) {
    std::unique_lock<std::mutex> lock(ex_control_mutex_);
    return ex_control_condition_.wait_for(lock, timeout, [this] { return ex_control_running_.load(); });
  }

  void publish_status(const std::string &status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published motion status: %s", status.c_str());
  }

  void activate_controller(const std::string& controller_name)
  {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {controller_name};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Activated controller '%s' (or already active)", controller_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Activation service call for '%s' timed out; continuing anyway", controller_name.c_str());
    }
  }

  void print_controller_status(const std::string& controller_name)
  {
    auto client = this->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "ListControllers service not available");
      return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      auto response = future.get();
      for (const auto& ctrl : response->controller) {
        if (ctrl.name == controller_name) {
          RCLCPP_INFO(this->get_logger(), "Controller '%s' state: %s",
                      ctrl.name.c_str(), ctrl.state.c_str());
          return;
        }
      }
      RCLCPP_WARN(this->get_logger(), "Controller '%s' not found", controller_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "ListControllers service call timed out");
    }
  }

  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // If we're already busy and configured to ignore, just log and return
    if (is_busy_ && ignore_if_busy_) {
      RCLCPP_WARN(this->get_logger(), "Received new goal but node is busy. Ignoring.");
      publish_status("busy");
      return;
    }

    // If we're busy and not ignoring, we could either queue or block.
    // For simplicity, we block until current motion finishes.
    if (is_busy_ && !ignore_if_busy_) {
      RCLCPP_INFO(this->get_logger(), "Received new goal, waiting for current motion to finish...");
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]{ return !is_busy_; });
    }

    // Set busy flag
    {
      std::lock_guard<std::mutex> lock(mutex_);
      is_busy_ = true;
    }

    RCLCPP_INFO(this->get_logger(),
      "Planning to pose: (%.2f, %.2f, %.2f), orientation: (%.2f, %.2f, %.2f, %.2f)",
      msg->position.x, msg->position.y, msg->position.z,
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w
    );

    std::thread([this, msg]()
    {
      try
      {
        // Wait for robot program to be running (timeout 5 seconds)
        if (!wait_for_ex_control_running()) {
          RCLCPP_ERROR(this->get_logger(), "Robot program not running. Please start External Control on the robot.");
          // Mark not busy and return early
          std::lock_guard<std::mutex> lock(ex_control_mutex_);
          is_busy_ = false;
          ex_control_condition_.notify_all();
          return;
        }
        
        // Activate controller
        publish_status("activating_controller");
        activate_controller("scaled_joint_trajectory_controller");
        print_controller_status("scaled_joint_trajectory_controller");
        
        // Plan motion
        publish_status("planning");
        auto move_group_interface =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), 
            planning_group_
          );
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // final goal accuracy
        move_group_interface->setGoalJointTolerance(0.1);        // rad
        move_group_interface->setGoalPositionTolerance(0.05);    // m
        move_group_interface->setGoalOrientationTolerance(0.1);  // rad
        // Set planning time (seconds)
        move_group_interface->setPlanningTime(30.0);
        // Set the planner ID from parameter (default is RRTConnectkConfigDefault)
        move_group_interface->setPlannerId(planner_id_);
        // Set the goal pose
        move_group_interface->setPoseTarget(*msg);

        // // joint goal test
        // std::vector<double> joint_goal = {0.0, -0.5, 0.5, -0.5, 0.5, 0.0};
        // move_group_interface->setJointValueTarget(joint_goal);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const success = static_cast<bool>(move_group_interface->plan(plan));

        if (success)
        {
          RCLCPP_INFO(this->get_logger(), "Planning succeeded.");
          if (execute_immediately_) 
          {
            publish_status("executing");
            RCLCPP_INFO(this->get_logger(), "Executing...");
            move_group_interface->execute(plan);
            publish_status("completed");
          } 
          else 
          {
            publish_status("completed");
            RCLCPP_INFO(this->get_logger(), "Execution disabled. (parameter execute_immediately=false)");
          }
        } 
        else 
        {
          RCLCPP_ERROR(this->get_logger(), "Planning failed for received pose.");
          publish_status("failed");
        }
        RCLCPP_INFO(this->get_logger(), 
          "Plan trajectory points: %zu",
          plan.trajectory_.joint_trajectory.points.size()
          );
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception in planning thread: %s", e.what());
        publish_status("failed");
      }

      // Mark as not busy and notify waiting threads
      {
        std::lock_guard<std::mutex> lock(mutex_);
        is_busy_ = false;
      }
      condition_.notify_all();
      publish_status("idle");
    }).detach();
  }

  void gripperCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty gripper command");
        return;
    }

    double width_m = msg->data[0];
    // Clamp to valid range (0.0 to 0.11 m for RG2)
    if (width_m < 0.0) width_m = 0.0;
    if (width_m > 0.11) width_m = 0.11;
    activate_controller("finger_width_trajectory_controller");

    auto gripper_group = 
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), 
        "ur_onrobot_gripper"
      );
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    gripper_group->setGoalJointTolerance(0.02);
    gripper_group->setJointValueTarget("finger_width", width_m);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (gripper_group->plan(plan))
    {
        gripper_group->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Gripper moved to width = %.3f m", width_m);
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "Failed to plan gripper motion to width %.3f m", width_m);
    }
  }

  std::atomic<bool> is_busy_;
  std::mutex mutex_;
  std::condition_variable condition_;

  std::atomic<bool> ex_control_running_{false};
  std::mutex ex_control_mutex_;
  std::condition_variable ex_control_condition_;
  
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ex_control_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3PlannerListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}