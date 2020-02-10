#ifndef PANDA_ARM_CONTROL_H
#define PANDA_ARM_CONTROL_H

#include <ros/console.h>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "ros/ros.h"

using namespace std;

namespace panda {

class Arm {
public:
  Arm ();
  void StringMessageCallback(const std_msgs::String::ConstPtr &msg);
  void PoseListenerCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void MoveArm();
  void MoveTargetPose(geometry_msgs::Pose target_pose, bool execute);
  void MoveTargetPoseCon(geometry_msgs::Pose target_pose, bool execute, double plan_time);
private:
  ros::NodeHandle nh_;
  int state; // ACTIVE, IDLE
  ros::Subscriber string_subscriber_;
  ros::Subscriber input_arm_state_;

  geometry_msgs::Pose home_pose_;
  std::vector<double> home_joint_;
  moveit_msgs::OrientationConstraint ocm_;

  moveit::planning_interface::MoveGroupInterface move_group_; //move_group object storing joint and link info

  std::string planning_group_ = "manipulator";

  bool success_ = false;
  bool joint_constraint_ = false;
  bool orient_constraint_ = false;

  // moveit::planning_interface::MoveGroupInterface move_group("manipulator"); //move_group object storing joint and link info
  moveit::planning_interface::MoveGroupInterface::Plan plan; //plan for motion planning
};
}

#endif //PANDA_ARM_CONTROL_H
