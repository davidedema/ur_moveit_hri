#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "motion",
    rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
  );



  // Create a ROS logger
  auto const logger = rclcpp::get_logger("motion");
  // do a 5 second sleep to wait for the moveit server to come up
  RCLCPP_INFO(logger, "Waiting for 5 seconds for moveit server to come up");
  rclcpp::sleep_for(std::chrono::seconds(5));

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "robot1ur5_manipulator");
  RCLCPP_INFO(logger, "Created MoveGroupInterface for 'robot1ur5_manipulator'");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.367;
    msg.position.y = 0.371;
    msg.position.z = 0.756;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);



  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
