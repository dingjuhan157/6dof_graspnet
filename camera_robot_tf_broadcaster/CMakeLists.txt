cmake_minimum_required(VERSION 3.0.2)
project(camera_robot_tf_broadcaster)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy tf2_ros geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/camera_robot_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
