cmake_minimum_required(VERSION 3.0.2)
project(jaka_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  jaka_msgs
  moveit_msgs
  actionlib
  actionlib_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jaka_motor_controller
  CATKIN_DEPENDS roscpp rospy std_msgs jaka_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

# 电机控制器库
add_library(jaka_motor_controller
  src/motor_controller.cpp
)

target_link_libraries(jaka_motor_controller
  ${catkin_LIBRARIES}
)

add_dependencies(jaka_motor_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 可执行文件
add_executable(moveit_server src/moveit_server.cpp)
add_executable(task1 src/task1.cpp)
add_executable(task2 src/task2.cpp)
add_executable(example src/example.cpp)  # 修正example的定义

# 添加依赖
add_dependencies(moveit_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(task1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(task2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库
target_link_libraries(moveit_server
  ${catkin_LIBRARIES}
  jakaAPI
)

target_link_libraries(task1
  jaka_motor_controller
  ${catkin_LIBRARIES}
  jakaAPI
)

target_link_libraries(task2
  jaka_motor_controller
  ${catkin_LIBRARIES}
  jakaAPI
)

target_link_libraries(example
  jaka_motor_controller
  ${catkin_LIBRARIES}
  jakaAPI
)

# 安装目标
install(TARGETS jaka_motor_controller task1 task2 moveit_server example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)