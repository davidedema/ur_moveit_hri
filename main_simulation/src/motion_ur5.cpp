#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_interfaces/srv/go_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"

#define TOPIC_PUB "/robot1/cartesian_motion_controller/target_frame"
#define TOPIC_SUB "/robot1/cartesian_motion_controller/current_pose"

using namespace std::chrono_literals;

class MotionManager : public rclcpp::Node
{
public:
  MotionManager()
      : Node("point_publisher"), current_pose_received_(false)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(TOPIC_PUB, 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TOPIC_SUB, 10, [this](geometry_msgs::msg::PoseStamped::UniquePtr msg)
        {
          if (!current_pose_received_)
          {
            RCLCPP_INFO(this->get_logger(), "Current pose received: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Current orientation received: [%f, %f, %f, %f]", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            current_pose_ = *msg;
            current_pose_received_ = true;
          } });
    server_ = this->create_service<bt_interfaces::srv::GoToPose>(
        "/robot1/go_to_pose", std::bind(&MotionManager::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    server_rise_ = this->create_service<std_srvs::srv::Trigger>(
        "/robot1/rise", std::bind(&MotionManager::rise_service, this, std::placeholders::_1, std::placeholders::_2));
  }

  bool is_current_pose_received() const
  {
    return current_pose_received_;
  }

  bool is_goal_pose_received() const
  {
    return goal_pose_received_;
  }

  void linear_interpolation(const geometry_msgs::msg::PoseStamped &goal_pose, int steps)
  {
    if (!current_pose_received_)
    {
      RCLCPP_WARN(this->get_logger(), "Current pose not received yet. Cannot perform interpolation.");
      return;
    }

    current_pose_received_ = false;
    goal_pose_received_ = false;

    RCLCPP_INFO(this->get_logger(), "Starting linear interpolation...");

    for (int i = 1; i <= steps; ++i)
    {
      geometry_msgs::msg::PoseStamped interpolated_pose;
      interpolated_pose.header.frame_id = "robot1base_link";
      interpolated_pose.header.stamp = this->get_clock()->now();

      // Linear interpolation for position
      interpolated_pose.pose.position.x = current_pose_.pose.position.x +
                                          (goal_pose.pose.position.x - current_pose_.pose.position.x) * i / steps;
      interpolated_pose.pose.position.y = current_pose_.pose.position.y +
                                          (goal_pose.pose.position.y - current_pose_.pose.position.y) * i / steps;
      interpolated_pose.pose.position.z = current_pose_.pose.position.z +
                                          (goal_pose.pose.position.z - current_pose_.pose.position.z) * i / steps;

      interpolated_pose.pose.orientation = goal_pose.pose.orientation;

      publisher_->publish(interpolated_pose);
      RCLCPP_INFO(this->get_logger(), "Published interpolated pose: [%f, %f, %f]",
                  interpolated_pose.pose.position.x, interpolated_pose.pose.position.y, interpolated_pose.pose.position.z);

      // Sleep to give time for the controller to process each pose
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Interpolation complete.");
  }

private:
  void handle_service(const std::shared_ptr<bt_interfaces::srv::GoToPose::Request> request,
                      std::shared_ptr<bt_interfaces::srv::GoToPose::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to go to pose: [%f, %f, %f, %d]",
                request->pose.position.x, request->pose.position.y, request->pose.position.z, request->rise_up);

    geometry_msgs::msg::PoseStamped goal_pose;
    bool rise_up = request->rise_up;
    goal_pose.header.frame_id = "robot1base_link";
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.pose = request->pose;
    if (rise_up)
    {
      RCLCPP_INFO(this->get_logger(), "Rising up");
      goal_pose.pose.position.z -= 0.15;
      this->linear_interpolation(goal_pose, 50);
      current_pose_received_ = true;
      current_pose_ = goal_pose;
      goal_pose.pose.position.z += 0.15;
      goal_pose.header.stamp = this->get_clock()->now();
    }
    this->linear_interpolation(goal_pose, 50);

    response->result = true;
    RCLCPP_INFO(this->get_logger(), "Published pose.");
  }

  void rise_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    geometry_msgs::msg::PoseStamped goal_pose = current_pose_;
    goal_pose.header.frame_id = "robot1base_link";
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.pose.position.z -= 0.15;
    RCLCPP_INFO(this->get_logger(), "Received request raising up from %f %f %f to %f %f %f",
                current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z,
                goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
    this->linear_interpolation(goal_pose, 50);
    RCLCPP_INFO(this->get_logger(), "Published pose.");
    response->success = true;
  }

  rclcpp::Service<bt_interfaces::srv::GoToPose>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_rise_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool current_pose_received_;
  bool goal_pose_received_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node_motion = std::make_shared<MotionManager>();

  while (rclcpp::ok())
  {
    // Wait for the current pose to be received
    RCLCPP_INFO(node_motion->get_logger(), "Waiting for the current pose ...");
    while (rclcpp::ok() && !node_motion->is_current_pose_received() && !node_motion->is_goal_pose_received())
    {
      rclcpp::spin_some(node_motion);
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_motion->get_logger(), "Waiting for the goal pose ...");
    while (rclcpp::ok() && !node_motion->is_goal_pose_received())
    {
      rclcpp::spin(node_motion);
      std::this_thread::sleep_for(100ms);
    }
  }

  rclcpp::shutdown();
  return 0;
}
