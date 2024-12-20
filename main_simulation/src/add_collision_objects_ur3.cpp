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

#define Z_BASE_LINK 1.79
#define Z_DESK 0.867

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
    wrist_1transform_stamped = tf_buffer_->lookupTransform("world", "robot2wrist_1_link", rclcpp::Time(0));
    wrist_2transform_stamped = tf_buffer_->lookupTransform("world", "robot2wrist_2_link", rclcpp::Time(0));
    wrist_3transform_stamped = tf_buffer_->lookupTransform("world", "robot2wrist_3_link", rclcpp::Time(0));
    shoulder_linktransform_stamped = tf_buffer_->lookupTransform("world", "robot2shoulder_link", rclcpp::Time(0));
    upper_armtransform_stamped = tf_buffer_->lookupTransform("world", "robot2upper_arm_link", rclcpp::Time(0));
    forearmtransform_stamped = tf_buffer_->lookupTransform("world", "robot2forearm_link", rclcpp::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(LOGGER, "Could not transform robot2wrist_1_link to world: %s", ex.what());
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
  upper_box.dimensions[0] = 0.4; // Length
  upper_box.dimensions[1] = 0.2;  // Width
  upper_box.dimensions[2] = 0.2;  // Height

  shape_msgs::msg::SolidPrimitive forearm_box;
  forearm_box.type = forearm_box.BOX;
  forearm_box.dimensions.resize(3);
  forearm_box.dimensions[0] = 0.3;  // Length
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
  pose_upper.position.y = upper_armtransform_stamped.transform.translation.y;
  pose_upper.position.z = upper_armtransform_stamped.transform.translation.z;

  pose_upper.orientation = upper_armtransform_stamped.transform.rotation;

  geometry_msgs::msg::Pose pose_fore;
  pose_fore.position.x = forearmtransform_stamped.transform.translation.x;
  pose_fore.position.y = forearmtransform_stamped.transform.translation.y+0.1;
  pose_fore.position.z = forearmtransform_stamped.transform.translation.z+0.1;

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

  // // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // // and stop before colliding
  // moveit_msgs::msg::CollisionObject desk;
  // moveit_msgs::msg::CollisionObject electric_panel;
  // moveit_msgs::msg::CollisionObject wall;
  // moveit_msgs::msg::CollisionObject top_plate;
  // moveit_msgs::msg::CollisionObject plug;

  // // define reference frame
  // desk.header.frame_id = "robot2base_link";
  // electric_panel.header.frame_id = "robot2base_link";
  // wall.header.frame_id = "robot2base_link";
  // top_plate.header.frame_id = "robot2base_link";
  // plug.header.frame_id = "robot2base_link";

  // desk.id = "desk";
  // electric_panel.id = "electric_panel";
  // wall.id = "wall";
  // top_plate.id = "top_plate";

  // shape_msgs::msg::SolidPrimitive primitive1, primitive2, primitive3, primitive4, primitive5;
  // // defining desk's collision
  // primitive1.type = primitive1.BOX;
  // primitive1.dimensions.resize(3);
  // primitive1.dimensions[0] = 0.85;
  // primitive1.dimensions[1] = 1.0;
  // primitive1.dimensions[2] = 0.8675;
  // // defining electric panel's collision
  // primitive2.type = primitive2.BOX;
  // primitive2.dimensions.resize(3);
  // primitive2.dimensions[0] = 0.175;
  // primitive2.dimensions[1] = 1;
  // primitive2.dimensions[2] = 0.175;
  // // defining wall's collision
  // primitive3.type = primitive3.BOX;
  // primitive3.dimensions.resize(3);
  // primitive3.dimensions[0] = 0.05;
  // primitive3.dimensions[1] = 1;
  // primitive3.dimensions[2] = 2;
  // // defining top plate's collision
  // primitive4.type = primitive4.BOX;
  // // primitive4.type = primitive3.BOX;
  // primitive4.dimensions.resize(3);
  // primitive4.dimensions[0] = 0.5;
  // primitive4.dimensions[1] = 1;
  // primitive4.dimensions[2] = 0.05;
  // // defining electric plug's collision
  // primitive5.type = primitive5.BOX;
  // primitive5.dimensions.resize(3);
  // primitive5.dimensions[0] = 0.1;
  // primitive5.dimensions[1] = 0.12;
  // primitive5.dimensions[2] = 0.09;

  // geometry_msgs::msg::Pose desk_pose, electric_panel_pose, wall_pose, top_plate_pose, plug_pose;
  // // defining desk's pose
  // desk_pose.orientation.w = -0.707;
  // desk_pose.orientation.z = 0.707;
  // desk_pose.position.x = -0.0;
  // desk_pose.position.y = 0.08;
  // desk_pose.position.z = Z_BASE_LINK - 0.867 / 2;
  // // defining electric panel's pose
  // electric_panel_pose.orientation.w = -0.707;
  // electric_panel_pose.orientation.z = 0.707;
  // electric_panel_pose.position.x = 0;
  // electric_panel_pose.position.y = -0.255;
  // electric_panel_pose.position.z = 0.92 - 0.075;
  // // defining wall's pose
  // wall_pose.orientation.z = 0.707;
  // wall_pose.orientation.w = -0.707;
  // wall_pose.position.x = 0;
  // wall_pose.position.y = -0.375;
  // wall_pose.position.z = 1;
  // // defining top plate's pose
  // top_plate_pose.orientation.z = 0.707;
  // top_plate_pose.orientation.w = -0.707;
  // top_plate_pose.position.x = 0;
  // top_plate_pose.position.y = -0.15;
  // top_plate_pose.position.z = -0.026;
  // // defining plug's pose
  // plug_pose.orientation.z = 0.707;
  // plug_pose.orientation.w = -0.707;
  // plug_pose.position.x = 0.44;
  // plug_pose.position.y = -0.2;
  // plug_pose.position.z = 0.92 - 0.06;

  // desk.primitives.push_back(primitive1);
  // desk.primitive_poses.push_back(desk_pose);
  // desk.operation = desk.ADD;

  // electric_panel.primitives.push_back(primitive2);
  // electric_panel.primitive_poses.push_back(electric_panel_pose);
  // electric_panel.operation = electric_panel.ADD;

  // wall.primitives.push_back(primitive3);
  // wall.primitive_poses.push_back(wall_pose);
  // wall.operation = wall.ADD;

  // top_plate.primitives.push_back(primitive4);
  // top_plate.primitive_poses.push_back(top_plate_pose);
  // top_plate.operation = top_plate.ADD;

  // plug.primitives.push_back(primitive5);
  // plug.primitive_poses.push_back(plug_pose);
  // plug.operation = plug.ADD;

  // psw.collision_objects.push_back(desk);
  // psw.collision_objects.push_back(electric_panel);
  // psw.collision_objects.push_back(wall);
  // psw.collision_objects.push_back(top_plate);
  // psw.collision_objects.push_back(plug);

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
  auto timer = node_->create_wall_timer(0.01s, [scene_pub]()
                                        { publish_moving_object(scene_pub); });

  // Multithreaded executor
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
