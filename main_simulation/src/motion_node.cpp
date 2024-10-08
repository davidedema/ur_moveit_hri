#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"


class MotionNode : public rclcpp::Node
{
public:
  MotionNode() : rclcpp::Node("motion_node")
  {
    this->subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/move_arm_string", 10, std::bind(&MotionNode::callback, this, std::placeholders::_1));

    this->last_pose.orientation.x = 0.0;
    this->last_pose.orientation.y = 0.0;
    this->last_pose.orientation.z = 0.0;
    this->last_pose.orientation.w = 0.0;
    this->last_pose.position.x = 0.0;
    this->last_pose.position.y = 0.0;
    this->last_pose.position.z = 0.0;
  }

  void callback(const std_msgs::msg::String &msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Pose safe_pose;
    geometry_msgs::msg::Pose home_pose;
    home_pose.orientation.x = -0.008744;
    home_pose.orientation.y = 0.038697;
    home_pose.orientation.z = 0.999046;
    home_pose.orientation.w = 0.018255;
    home_pose.position.x = -0.45;
    home_pose.position.y = 0.37;
    home_pose.position.z = 0.60;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    if (msg.data == "v1")
    {
      target_pose.orientation.x = -0.008744;
      target_pose.orientation.y = 0.038697;
      target_pose.orientation.z = 0.999046;
      target_pose.orientation.w = 0.018255;
      target_pose.position.x = 0.367;
      target_pose.position.y = 0.371;
      target_pose.position.z = 0.756;
    }
    else if (msg.data == "v2")
    {
      target_pose.orientation.x = -0.008744;
      target_pose.orientation.y = 0.038697;
      target_pose.orientation.z = 0.999046;
      target_pose.orientation.w = 0.018255;
      target_pose.position.x = 0.204;
      target_pose.position.y = 0.362;
      target_pose.position.z = 0.756;
    }
    else if (msg.data == "v3")
    {
      target_pose.orientation.x = -0.008744;
      target_pose.orientation.y = 0.038697;
      target_pose.orientation.z = 0.999046;
      target_pose.orientation.w = 0.018255;
      target_pose.position.x = 0.035;
      target_pose.position.y = 0.362;
      target_pose.position.z = 0.756;
    }
    else if (msg.data == "s1")
    {
      target_pose.orientation.x = -0.013393;
      target_pose.orientation.y = 0.031382;
      target_pose.orientation.z = 0.690826;
      target_pose.orientation.w = 0.722216;
      target_pose.position.x = -0.399;
      target_pose.position.y = 0.305;
      target_pose.position.z = 0.756;
    }
    else if (msg.data == "s2")

    {
      target_pose.orientation.x = -0.013393;
      target_pose.orientation.y = 0.031382;
      target_pose.orientation.z = 0.690826;
      target_pose.orientation.w = 0.722216;
      target_pose.position.x = -0.437;
      target_pose.position.y = 0.152;
      target_pose.position.z = 0.756;
    }
    else if (msg.data == "s3")
    {
      target_pose.orientation.x = -0.013393;
      target_pose.orientation.y = 0.031382;
      target_pose.orientation.z = 0.690826;
      target_pose.orientation.w = 0.722216;
      target_pose.position.x = -0.450;
      target_pose.position.y = 0.015;
      target_pose.position.z = 0.756;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid command");
    }

    if (msg.data == "home") {
      waypoints.push_back(home_pose);
    }
    else {
      safe_pose = target_pose;
      safe_pose.position.z = this->safe_z;
      waypoints.push_back(safe_pose);
      waypoints.push_back(target_pose);
    }

    this->move(waypoints);
  }

  void move(std::vector<geometry_msgs::msg::Pose> &waypoints)
  {
    auto this_shared_pointer = this->shared_from_this();
    moveit::planning_interface::MoveGroupInterface move_group(this_shared_pointer, "ur_manipulator");

    // Get the current pose
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
    RCLCPP_INFO(this->get_logger(), "Current pose: x=%f, y=%f, z=%f, Current orientation: x=%f, y=%f, z=%f, w=%f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z,
                start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);

    if (!uninitialized_last_pose()){
      RCLCPP_INFO(this->get_logger(), "Initialized, so going up");
      std::vector<geometry_msgs::msg::Pose> temp_waypoints;
      this->last_pose.position.z = this->safe_z;
      temp_waypoints.push_back(this->last_pose);
      moveit::planning_interface::MoveGroupInterface::Plan temp_cartesian_plan;
      double temp_fraction = move_group.computeCartesianPath(temp_waypoints, 0.01, 10.0, temp_cartesian_plan.trajectory_);
      RCLCPP_INFO(this->get_logger(), "[TEMP] Cartesian path planning result: %f%%", temp_fraction * 100.0);
      RCLCPP_INFO(this->get_logger(), "[TEMP] Points to follow %d", temp_cartesian_plan.trajectory_.joint_trajectory.points.size());
      move_group.execute(temp_cartesian_plan);
    }

    // Compute Cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 10.0, cartesian_plan.trajectory_);

    RCLCPP_INFO(this->get_logger(), "Cartesian path planning result: %f%%", fraction * 100.0);
    RCLCPP_INFO(this->get_logger(), "Points to follow %d", cartesian_plan.trajectory_.joint_trajectory.points.size());

    this->last_pose = waypoints.back();

    // Optionally, execute the plan
    move_group.execute(cartesian_plan);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  double safe_z = 0.65;
  geometry_msgs::msg::Pose last_pose;

  bool uninitialized_last_pose()
  {
    return this->last_pose.orientation.x == 0.0 && this->last_pose.orientation.y == 0.0 && this->last_pose.orientation.z == 0.0 &&
           this->last_pose.orientation.w == 0.0 && this->last_pose.position.x == 0.0 && this->last_pose.position.y == 0.0 && this->last_pose.position.z == 0.0;
  }
};

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<MotionNode>();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
