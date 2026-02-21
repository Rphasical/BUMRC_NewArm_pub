cmake_minimum_required(VERSION 3.8)
project(arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17 (modern ROS 2 standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(rviz2 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(canopen_core REQUIRED)
find_package(linkattacher_msgs REQUIRED)  
find_package(ros2_linkattacher REQUIRED)


# Example executables (add your nodes here if needed)
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp moveit_core moveit_ros_planning_interface ...)

# Install URDF, XACRO, meshes, launch files, RViz configs, and worlds
install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cam_files
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})


add_executable(usb_place src/usb_place.cpp)
ament_target_dependencies(usb_place rclcpp moveit_ros_planning_interface sensor_msgs geometry_msgs linkattacher_msgs ros2_linkattacher pcl_conversions pcl_ros)


install(TARGETS 
usb_place
  DESTINATION lib/${PROJECT_NAME})
# Install Python scripts if you have any
# install(PROGRAMS
#   scripts/my_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/cpplint if needed
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
