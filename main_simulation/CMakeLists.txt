cmake_minimum_required(VERSION 3.8)
project(main_simulation)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(collision_loader_node src/add_collision_objects.cpp)

ament_target_dependencies(collision_loader_node
    rclcpp
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
)

install(TARGETS
    collision_loader_node
    DESTINATION lib/${PROJECT_NAME})


add_executable(motion_node src/motion_node.cpp)

add_executable(poc_motion src/example_motion.cpp)

ament_target_dependencies(motion_node
    geometry_msgs
    rclcpp
    std_msgs
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
)

ament_target_dependencies(poc_motion
    geometry_msgs
    rclcpp
    std_msgs
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
)

target_compile_features(motion_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(poc_motion PUBLIC c_std_99 cxx_std_17)


install(TARGETS
    motion_node
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    poc_motion
    DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch urdf rviz config srdf scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gripper_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/publisher_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
