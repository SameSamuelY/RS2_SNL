#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
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

    // Create subscriber
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/ur3_goal_pose", 10,
      std::bind(&Ur3PlannerListener::poseCallback, this, std::placeholders::_1)
    );

    status_publisher_ = this->create_publisher<std_msgs::msg::String>("/motion_status", 10);
    publish_status("idle");

    RCLCPP_INFO(this->get_logger(), "Node started. Listening for poses on /ur3_goal_pose");
    RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());
  }

private:
  void publish_status(const std::string &status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published motion status: %s", status.c_str());
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
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    std::thread([this, msg]() {
      try
      {
        // Activate controller
        publish_status("activating_controller");
        auto client = this->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = {"scaled_joint_trajectory_controller"};
        client->async_send_request(request);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        print_controller_status("scaled_joint_trajectory_controller");
        
        publish_status("planning");
        auto move_group_interface =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), 
            planning_group_
          );
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        // Set planning time (seconds)
        move_group_interface->setPlanningTime(30.0);
        // Set the planner ID from parameter (default is RRTConnectkConfigDefault)
        move_group_interface->setPlannerId(planner_id_);
        // Set the goal pose
        move_group_interface->setPoseTarget(*msg);

        // // joint goal test
        // std::vector<double> joint_goal = {0.0, -0.5, 0.5, -0.5, 0.5, 0.0};
        // move_group_interface->setJointValueTarget(joint_goal);

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const success = static_cast<bool>(move_group_interface->plan(plan));

        if (success) {
          RCLCPP_INFO(this->get_logger(), "Planning succeeded.");
          if (execute_immediately_) {
            publish_status("executing");
            RCLCPP_INFO(this->get_logger(), "Executing...");
            move_group_interface->execute(plan);
            publish_status("completed");
          } else {
            publish_status("completed");
            RCLCPP_INFO(this->get_logger(), "Execution disabled. (parameter execute_immediately=false)");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Planning failed for received pose.");
          publish_status("failed");
        }

        RCLCPP_INFO(this->get_logger(), "Plan trajectory points: %zu",
          plan.trajectory_.joint_trajectory.points.size());
      }
      catch (const std::exception& e) {
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

  std::string planning_group_;
  bool execute_immediately_;
  bool ignore_if_busy_;
  std::string planner_id_;
  
  std::atomic<bool> is_busy_;
  std::mutex mutex_;
  std::condition_variable condition_;
  
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ur3PlannerListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}