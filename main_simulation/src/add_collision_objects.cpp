#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_object");
rclcpp::Node::SharedPtr node_;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

void publish_moving_object(rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub)
{
  // Query the transform of the robot arm's base link
  geometry_msgs::msg::TransformStamped wrist_1transform_stamped;
  geometry_msgs::msg::TransformStamped wrist_2transform_stamped;
  geometry_msgs::msg::TransformStamped wrist_3transform_stamped;
  geometry_msgs::msg::TransformStamped shoulder_linktransform_stamped;
  geometry_msgs::msg::TransformStamped upper_armtransform_stamped;
  geometry_msgs::msg::TransformStamped forearmtransform_stamped;
  try
  {
    wrist_1transform_stamped = tf_buffer_->lookupTransform("world", "robot1wrist_1_link", rclcpp::Time(0));
    wrist_2transform_stamped = tf_buffer_->lookupTransform("world", "robot1wrist_2_link", rclcpp::Time(0));
    wrist_3transform_stamped = tf_buffer_->lookupTransform("world", "robot1wrist_3_link", rclcpp::Time(0));
    shoulder_linktransform_stamped = tf_buffer_->lookupTransform("world", "robot1shoulder_link", rclcpp::Time(0));
    upper_armtransform_stamped = tf_buffer_->lookupTransform("world", "robot1upper_arm_link", rclcpp::Time(0));
    forearmtransform_stamped = tf_buffer_->lookupTransform("world", "robot1forearm_link", rclcpp::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(LOGGER, "Could not transform robot1wrist_1_link to world: %s", ex.what());
    return;
  }

  //----------------------------------------------------------------

  // Create and define the collision object
  moveit_msgs::msg::CollisionObject wrist_1_object;
  wrist_1_object.header.frame_id = "world";
  wrist_1_object.id = "wrist_1_object";

  moveit_msgs::msg::CollisionObject wrist_2_object;
  wrist_2_object.header.frame_id = "world";
  wrist_2_object.id = "wrist_2_object";

  moveit_msgs::msg::CollisionObject wrist_3_object;
  wrist_3_object.header.frame_id = "world";
  wrist_3_object.id = "wrist_3_object";

  moveit_msgs::msg::CollisionObject shoulder_object;
  shoulder_object.header.frame_id = "world";
  shoulder_object.id = "shoulder_object";

  moveit_msgs::msg::CollisionObject upper_object;
  upper_object.header.frame_id = "world";
  upper_object.id = "upper_object";

  moveit_msgs::msg::CollisionObject forearm_object;
  forearm_object.header.frame_id = "world";
  forearm_object.id = "forearm_object";

  //----------------------------------------------------------------

  shape_msgs::msg::SolidPrimitive wrist_1;
  wrist_1.type = wrist_1.BOX;
  wrist_1.dimensions.resize(3);
  wrist_1.dimensions[0] = 0.15; // Length
  wrist_1.dimensions[1] = 0.15; // Width
  wrist_1.dimensions[2] = 0.25; // Height

  shape_msgs::msg::SolidPrimitive wrist_2;
  wrist_2.type = wrist_2.BOX;
  wrist_2.dimensions.resize(3);
  wrist_2.dimensions[0] = 0.15; // Length
  wrist_2.dimensions[1] = 0.15; // Width
  wrist_2.dimensions[2] = 0.25; // Height

  shape_msgs::msg::SolidPrimitive wrist_3;
  wrist_3.type = wrist_3.BOX;
  wrist_3.dimensions.resize(3);
  wrist_3.dimensions[0] = 0.15; // Length
  wrist_3.dimensions[1] = 0.15; // Width
  wrist_3.dimensions[2] = 0.25; // Height

  shape_msgs::msg::SolidPrimitive shoulder_box;
  shoulder_box.type = shoulder_box.BOX;
  shoulder_box.dimensions.resize(3);
  shoulder_box.dimensions[0] = 0.15; // Length
  shoulder_box.dimensions[1] = 0.25; // Width
  shoulder_box.dimensions[2] = 0.15; // Height

  shape_msgs::msg::SolidPrimitive upper_box;
  upper_box.type = upper_box.BOX;
  upper_box.dimensions.resize(3);
  upper_box.dimensions[0] = 0.60; // Length
  upper_box.dimensions[1] = 0.2; // Width
  upper_box.dimensions[2] = 0.2; // Height

  shape_msgs::msg::SolidPrimitive forearm_box;
  forearm_box.type = forearm_box.BOX;
  forearm_box.dimensions.resize(3);
  forearm_box.dimensions[0] = 0.6; // Length
  forearm_box.dimensions[1] = 0.15; // Width
  forearm_box.dimensions[2] = 0.15; // Height

  //----------------------------------------------------------------

  geometry_msgs::msg::Pose pose_wrist1;
  pose_wrist1.position.x = wrist_1transform_stamped.transform.translation.x;
  pose_wrist1.position.y = wrist_1transform_stamped.transform.translation.y;
  pose_wrist1.position.z = wrist_1transform_stamped.transform.translation.z;

  pose_wrist1.orientation = wrist_1transform_stamped.transform.rotation;

  geometry_msgs::msg::Pose pose_wrist2;
  pose_wrist2.position.x = wrist_2transform_stamped.transform.translation.x;
  pose_wrist2.position.y = wrist_2transform_stamped.transform.translation.y;
  pose_wrist2.position.z = wrist_2transform_stamped.transform.translation.z;

  pose_wrist2.orientation = wrist_2transform_stamped.transform.rotation;

  geometry_msgs::msg::Pose pose_wrist3;
  pose_wrist3.position.x = wrist_3transform_stamped.transform.translation.x;
  pose_wrist3.position.y = wrist_3transform_stamped.transform.translation.y;
  pose_wrist3.position.z = wrist_3transform_stamped.transform.translation.z;

  pose_wrist3.orientation = wrist_3transform_stamped.transform.rotation;

  geometry_msgs::msg::Pose pose_shoulder;
  pose_shoulder.position.x = shoulder_linktransform_stamped.transform.translation.x;
  pose_shoulder.position.y = shoulder_linktransform_stamped.transform.translation.y;
  pose_shoulder.position.z = shoulder_linktransform_stamped.transform.translation.z;

  pose_shoulder.orientation = shoulder_linktransform_stamped.transform.rotation;

  geometry_msgs::msg::Pose pose_upper;
  pose_upper.position.x = upper_armtransform_stamped.transform.translation.x;
  pose_upper.position.y = upper_armtransform_stamped.transform.translation.y+0.18;
  pose_upper.position.z = upper_armtransform_stamped.transform.translation.z-0.18;

  pose_upper.orientation = upper_armtransform_stamped.transform.rotation;
  
  geometry_msgs::msg::Pose pose_fore;
  pose_fore.position.x = forearmtransform_stamped.transform.translation.x;
  pose_fore.position.y = forearmtransform_stamped.transform.translation.y;
  pose_fore.position.z = forearmtransform_stamped.transform.translation.z+0.2;

  pose_fore.orientation = forearmtransform_stamped.transform.rotation;
  //----------------------------------------------------------------

  // Create the planning scene message
  moveit_msgs::msg::PlanningSceneWorld psw;

  wrist_1_object.primitives.push_back(wrist_1);
  wrist_1_object.primitive_poses.push_back(pose_wrist1);
  wrist_1_object.operation = wrist_1_object.ADD;
  psw.collision_objects.push_back(wrist_1_object);

  wrist_2_object.primitives.push_back(wrist_2);
  wrist_2_object.primitive_poses.push_back(pose_wrist2);
  wrist_2_object.operation = wrist_2_object.ADD;
  psw.collision_objects.push_back(wrist_2_object);

  wrist_3_object.primitives.push_back(wrist_3);
  wrist_3_object.primitive_poses.push_back(pose_wrist3);
  wrist_3_object.operation = wrist_3_object.ADD;
  psw.collision_objects.push_back(wrist_3_object);

  shoulder_object.primitives.push_back(shoulder_box);
  shoulder_object.primitive_poses.push_back(pose_shoulder);
  shoulder_object.operation = shoulder_object.ADD;
  psw.collision_objects.push_back(shoulder_object);

  upper_object.primitives.push_back(upper_box);
  upper_object.primitive_poses.push_back(pose_upper);
  upper_object.operation = upper_object.ADD;
  psw.collision_objects.push_back(upper_object);

  forearm_object.primitives.push_back(forearm_box);
  forearm_object.primitive_poses.push_back(pose_fore);
  forearm_object.operation = forearm_object.ADD;
  psw.collision_objects.push_back(forearm_object);

  //----------------------------------------------------------------

  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the updated planning scene
  // RCLCPP_INFO(LOGGER, "Publishing object at Pose: [X: %f, Y: %f, Z: %f]",
  //             pose.position.x, pose.position.y, pose.position.z);
  scene_pub->publish(ps);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("dynamic_object_node", node_options);

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

  // Timer to update the object's position every second
  auto timer = node_->create_wall_timer(1s, [scene_pub]()
                                        { publish_moving_object(scene_pub); });

  // Multithreaded executor
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
