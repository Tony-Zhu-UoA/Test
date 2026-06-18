cmake_minimum_required(VERSION 3.0.2)
project(lab_codes)

# Find catkin and any other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  actionlib
  controller_manager_msgs
  control_msgs
  trajectory_msgs
  robotiq_85_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs actionlib controller_manager_msgs control_msgs trajectory_msgs avoidance_functions
)

## Enable Python setup
catkin_python_setup()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/avoidance_scripts/transforms/camera_transform.py
  scripts/avoidance_scripts/full_body_publisher.py
  scripts/avoidance_scripts/joint_position_movement.py
  scripts/avoidance_scripts/proximity_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
