#ifndef PANDA_ARM_CONTROL_H
#define PANDA_ARM_CONTROL_H

#include <ros/console.h>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include "roboperation/ArmStatePose.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/RobotTrajectory.h>

#include "ros/ros.h"
#include <math.h>
#include <queue>

// #include <franka>

/*
How to get Serial Communication from Arduino working:
- Modify rosserial_arduino/src/ArduinoHardware.h to include these lines
(https://answers.ros.org/question/264764/rosserial-arduino-due-sync-issues/)


# ROS TO ARDUINO
# terminal 1:$ roscore
# $ sudo chmod a+rw /dev/ttyACM0
# $ sudo usermod -a -G dialout amalaran
# NOTE: MAKE SURE THE SERIAL PORT WINDOW ON ARDUINO IS CLOSED
# terminal 2:$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=9600
# terminal 3:$ rostopic echo chatter
# terminal 4:$ rostopic echo arm_control_input

*/

using namespace std;
namespace rvt = rviz_visual_tools;

constexpr bool EXECUTE_MOTION = true;

// If the euclidean distance between the current pose and the desired pose
// is greater than this value, we invalidate it
constexpr double small_distance_threshold = 10.0;
// Boundaries
vector<double> x_bounds = {0, 0.55};
vector<double> y_bounds = {0, 0.60};
vector<double> z_bounds = {0.30, 1.0};

namespace panda {

class Arm {
private:
  ros::NodeHandle nh_;
  int state; // ACTIVE, IDLE
  ros::Subscriber string_subscriber_;
  ros::Subscriber input_arm_state_;
  ros::Subscriber arm_control_input_;

  ros::Publisher robot_arm_error_publisher_;

  geometry_msgs::Pose home_pose_;
  std::vector<double> home_joint_;
  moveit_msgs::OrientationConstraint ocm_;

  moveit_msgs::RobotTrajectory trajectory;

  // moveit::planning_interface::MoveGroupInterface move_group_; //move_group object storing joint and link info

  std::string planning_group_ = "panda_arm";

  bool success_ = false;
  bool joint_constraint_ = false;
  bool orient_constraint_ = false;

  moveit::planning_interface::MoveGroupInterface move_group_; //move_group object storing joint and link info
  const robot_state::JointModelGroup *joint_model_group_;     //pointer to joint_model_group
  moveit_visual_tools::MoveItVisualTools visual_tools; //visual tools to use the GUI in rviz
  moveit::planning_interface::MoveGroupInterface::Plan plan_; //plan for motion planning
  queue<roboperation::ArmStatePose> action_buffer_;
  geometry_msgs::Pose most_recent_goal_state_;

public:
  Arm ();
  void SetPubsAndSubs();
  void Reset();
  void Loop();
  void PrintPose(geometry_msgs::Pose pose_values);
  void PrintJointValues(std::vector<double> joint_values);
  void StringMessageCallback(const std_msgs::String::ConstPtr &msg);
  void PoseListenerCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void setMaxVelScalingFactor(double velocity_factor);
  bool ValidateTargetPose(geometry_msgs::Pose pose);
  double getSquaredEuclideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

  void MoveArmRealTimeCallback(const roboperation::ArmStatePose::ConstPtr &msg);

  void VisualizeTrajectory();
  void MoveTargetPose(geometry_msgs::Pose target_pose, bool execute);
  void MoveTargetPoseCon(geometry_msgs::Pose target_pose, bool execute, double plan_time);
  void MoveTargetJoint(std::vector<double> target_joint, bool execute);

  void MoveArmDirectionX(double distance);
  void MoveArmDirectionY(double distance);
  void MoveArmDirectionZ(double distance);

  void moveCartesianPath(double jump_threshold = 0.0, double eef_step = 0.01, bool step = true, bool execute = false);
  void executeCartesianPath(std::vector<geometry_msgs::Pose> waypoints, bool execute);
  void ExecuteBufferedActions();
  void ImpedanceController();
  void TestMovements();
};
}

#endif //PANDA_ARM_CONTROL_H
