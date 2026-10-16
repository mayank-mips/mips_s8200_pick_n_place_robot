cmake_minimum_required(VERSION 3.0.2)
project(mips_pick_n_place)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs tf2_ros cv_bridge image_transport gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python nodes
catkin_install_python(PROGRAMS
  scripts/object_detector.py
  scripts/motion_ik.py
  scripts/pick_and_place_fsm.py
  scripts/vacuum_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install resource directories
install(DIRECTORY launch urdf worlds config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
