#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
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
    }

    void callback(const std_msgs::msg::String & msg){
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

        geometry_msgs::msg::Pose pose;
        tf2::Quaternion myQuaternionS;
        tf2::Quaternion myQuaternionV;
        myQuaternionS.setRPY(1.57, -1.57, 0);
        myQuaternionV.setRPY(0, -1.57, 0);
        if (msg.data == "v1")
        {
            pose.orientation.x = myQuaternionV.x();
            pose.orientation.y = myQuaternionV.y();
            pose.orientation.z = myQuaternionV.z();
            pose.orientation.w = myQuaternionV.w();
            pose.position.x = -0.15;
            pose.position.y = 0.35;
            pose.position.z = 0.75;
        }
        else if (msg.data == "v2")
        {
            pose.orientation.x = myQuaternionV.x();
            pose.orientation.y = myQuaternionV.y();
            pose.orientation.z = myQuaternionV.z();
            pose.orientation.w = myQuaternionV.w();
            pose.position.x = -0.3;
            pose.position.y = 0.35;
            pose.position.z = 1.05;
        }
        else if (msg.data == "v3")
        {
            pose.orientation.x = myQuaternionV.x();
            pose.orientation.y = myQuaternionV.y();
            pose.orientation.z = myQuaternionV.z();
            pose.orientation.w = myQuaternionV.w();
            pose.position.x = -0.45;
            pose.position.y = 0.35;
            pose.position.z = 0.75;
        }
        else if (msg.data == "s1")
        {
            pose.orientation.x = myQuaternionS.x();
            pose.orientation.y = myQuaternionS.y();
            pose.orientation.z = myQuaternionS.z();
            pose.orientation.w = myQuaternionS.w();
            pose.position.x = -0.4;
            pose.position.y = 0.2;
            pose.position.z = 0.75;
        }
        else if (msg.data == "s2")
        {
            pose.orientation.x = myQuaternionS.x();
            pose.orientation.y = myQuaternionS.y();
            pose.orientation.z = myQuaternionS.z();
            pose.orientation.w = myQuaternionS.w();
            pose.position.x = -0.4;
            pose.position.y = 0.1;
            pose.position.z = 0.75;
        }
        else if (msg.data == "s3")
        {
            pose.orientation.x = myQuaternionS.x();
            pose.orientation.y = myQuaternionS.y();
            pose.orientation.z = myQuaternionS.z();
            pose.orientation.w = myQuaternionS.w();
            pose.position.x = -0.4;
            pose.position.y = 0.0;
            pose.position.z = 0.75;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid command");
        }

        this->move(pose);
    }


    void move(const geometry_msgs::msg::Pose &msg)
    {
        // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;

        auto node_interface = this->shared_from_this();

        auto move_group_interface = MoveGroupInterface(node_interface, "ur_manipulator");

        move_group_interface.setPoseTarget(msg);

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if (success)
        {
            move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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