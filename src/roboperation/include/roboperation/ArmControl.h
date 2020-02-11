#ifndef PANDA_ARM_CONTROL_H
#define PANDA_ARM_CONTROL_H

#include <ros/console.h>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros/ros.h"

using namespace std;

namespace panda {

class Arm {
private:
  ros::NodeHandle nh_;
  int state; // ACTIVE, IDLE
  ros::Subscriber string_subscriber_;
  ros::Subscriber input_arm_state_;

  ros::Publisher robot_arm_error_publisher_;

  geometry_msgs::Pose home_pose_;
  std::vector<double> home_joint_;
  moveit_msgs::OrientationConstraint ocm_;

  // moveit::planning_interface::MoveGroupInterface move_group_; //move_group object storing joint and link info

  std::string planning_group_ = "panda_arm";

  bool success_ = false;
  bool joint_constraint_ = false;
  bool orient_constraint_ = false;

  moveit::planning_interface::MoveGroupInterface move_group_; //move_group object storing joint and link info
  moveit::planning_interface::MoveGroupInterface::Plan plan_; //plan for motion planning
public:
  Arm ();
  void PrintCurrentPose(geometry_msgs::Pose pose_values);
  void PrintCurrentJointValues(std::vector<double> joint_values);
  void StringMessageCallback(const std_msgs::String::ConstPtr &msg);
  void PoseListenerCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void setMaxVelScalingFactor(double velocity_factor);
  bool ValidateTargetPose(geometry_msgs::Pose pose);
  void MoveTargetPose(geometry_msgs::Pose target_pose, bool execute);
  void MoveTargetPoseCon(geometry_msgs::Pose target_pose, bool execute, double plan_time);
  void MoveTargetJoint(std::vector<double> target_joint, bool execute);

  void moveCartesianPath(double jump_threshold = 0.0, double eef_step = 0.01, bool step = true, bool execute = false);
  void executeCartesianPath(std::vector<geometry_msgs::Pose> waypoints);
};
}

#endif //PANDA_ARM_CONTROL_H
