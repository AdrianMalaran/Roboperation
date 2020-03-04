cmake_minimum_required(VERSION 2.8.3)
project(roboperation)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
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
  eigen_conversions
  franka_control
  franka_hw
  franka_description
  realtime_tools
  pluginlib
  tf
  tf_conversions
  hardware_interface
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_message_files(
  FILES
  ArmStatePose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/compliance_param.cfg
#)

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
    franka_hw
    realtime_tools
    hardware_interface
    controller_interface
    dynamic_reconfigure
  DEPENDS
    system_lib
    Franka
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(roboperation
  src/ArmControl.cpp
  include/roboperation/ArmControl.h
  src/roboperation_example_controller.cpp
  include/roboperation/roboperation_example_controller.h
)

#add_executable(roboperation
#  src/main.cpp
#)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(roboperation
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(roboperation SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(roboperation PUBLIC
  include
)

#############
## Install ##
#############
install(TARGETS roboperation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES roboperation_example_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
