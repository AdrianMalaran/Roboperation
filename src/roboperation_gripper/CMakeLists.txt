cmake_minimum_required(VERSION 2.8.3)
project(roboperation_gripper)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  roscpp
  rospy
  geometry_msgs
  message_generation
  control_msgs
  actionlib
  sensor_msgs
  xmlrpcpp
  actionlib_msgs
  franka_control
  franka_hw
)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES Grasp.action
        Homing.action
        Stop.action
        Move.action
)

add_message_files(
  DIRECTORY msg
  FILES GraspEpsilon.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboperation_gripper
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    roscpp
    rospy
    geometry_msgs
    message_runtime
    control_msgs
    actionlib
    sensor_msgs
    xmlrpcpp
    actionlib_msgs
    franka_hw
  DEPENDS
    system_lib
    Franka
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(roboperation_gripper
  # Gripper
  src/roboperation_gripper.cpp
)

add_executable(roboperation_gripper_node
  src/roboperation_gripper_node.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

add_dependencies(roboperation_gripper_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  roboperation_gripper
)

target_link_libraries(roboperation_gripper
   ${Franka_LIBRARIES}
   ${catkin_LIBRARIES}
)

target_link_libraries(roboperation_gripper_node
  ${catkin_LIBRARIES}
  roboperation_gripper
)

target_include_directories(roboperation_gripper_node SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(roboperation_gripper PUBLIC
  include
)

## Installation
install(TARGETS roboperation_gripper
                roboperation_gripper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
