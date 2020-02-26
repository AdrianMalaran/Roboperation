cmake_minimum_required(VERSION 2.8.3)
project(roboperation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  rosbag
  message_generation
)

add_message_files(
  FILES
  ArmStatePose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboperation
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(arm_control
  src/ArmControl.cpp
  include/roboperation/ArmControl.h
)

add_executable(roboperation
  src/main.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(roboperation arm_control ${catkin_LIBRARIES})

#############
## Install ##
#############
