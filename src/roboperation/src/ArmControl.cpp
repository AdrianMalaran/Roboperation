#include "roboperation/ArmControl.h"

namespace panda {

Arm::Arm()
: move_group_(planning_group_)
{
  ROS_INFO("Initializing Arm Control.");

  // Subscribers
  input_arm_state_ = nh_.subscribe("input_state", 1, &Arm::PoseListenerCallback, this);
  string_subscriber_ = nh_.subscribe("chatter", 1, &Arm::StringMessageCallback, this);

  // Publishers
  robot_arm_error_publisher_ = nh_.advertise<std_msgs::String>("/error", 1, false);

  // moveit::planning_interface::MoveGroupInterface move_group_(planning_group_); //move_group_ object storing joint and link info

  static const std::string PLANNING_GROUP = "panda_arm";

  // Set the max velocity scaling factor
  setMaxVelScalingFactor(0.5);

  // moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup* joint_model_group =
  // move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // move_group.getCurrentState();
  home_pose_ = move_group_.getCurrentPose().pose;
  home_joint_ = move_group_.getCurrentJointValues();

  geometry_msgs::Pose test_goal_pose;
  test_goal_pose.position.x = 0.18;
  test_goal_pose.position.y = 0.51;
  test_goal_pose.position.z = 0.50;

  std::vector<double> test_goal_joints1 =
    {0.301032, 0.428689, 1.150879, -2.100787, 2.670032, 0.879070, 0.00};
  std::vector<double> test_goal_joints2 =
    {0.301032, 0.428689, 1.150879, -2.100787, 2.670032, 0.879070, 1.00};

  //Execute Motion
  // MoveTargetPose(test_goal_pose, true);
  // MoveTargetJoint(test_goal_joints1, true);
  // MoveTargetJoint(test_goal_joints2, true);

  // To Do List:
  // Test WayPoints
  std::vector<geometry_msgs::Pose> test_waypoints;
  for (int i = 0; i <= 10; i++) {
    geometry_msgs::Pose point;
    point.position.x = 0.20;
    point.position.y = 0.40 + i;
    point.position.z = 0.60;
    test_waypoints.push_back(point);
  }

  ROS_INFO("Testing Waypoints:");
  for (geometry_msgs::Pose point : test_waypoints) {
    PrintCurrentPose(point);
  }

  while (ros::ok()) {
    PrintCurrentPose(move_group_.getCurrentPose().pose);
    PrintCurrentJointValues(move_group_.getCurrentJointValues());
    ros::Duration(0.5).sleep();
  }
}

void Arm::PrintCurrentPose(geometry_msgs::Pose pose_values) {
  ROS_INFO("Point: Pos(%.2f, %.2f, %.2f)", pose_values.position.x, pose_values.position.y, pose_values.position.z);
}

void Arm::PrintCurrentJointValues(std::vector<double> joint_values) {
  std::string joints;
  for (double joint : joint_values) {
    joints += to_string(joint) + ", ";
  }
  ROS_INFO("Joints (%s)", joints.c_str());
}

void Arm::StringMessageCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received String: %s", msg->data.c_str());
}

void Arm::PoseListenerCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  ROS_INFO("Received Desired Pos(%.2f, %.2f, %.2f) Angle(%.2f, %.2f, %.2f)",
    msg->position.x, msg->position.y, msg->position.z,
    msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void Arm::setMaxVelScalingFactor(double velocity_factor) {
    ROS_INFO("Setting Velocity Factor: %.2f", velocity_factor);
    move_group_.setMaxVelocityScalingFactor(min(velocity_factor, 1.0));
    // vel_factor = velocity_factor;
}

bool Arm::ValidateTargetPose(geometry_msgs::Pose pose) {
  // Boundaries
  // x: [0 - 0.50]
  // y: [0 - 0.50]
  // z: [0.30 - 1.0]
  bool x_inbound = (pose.position.x >= 0 && pose.position.x <= 0.50);
  bool y_inbound = (pose.position.y >= 0 && pose.position.y <= 0.60);
  bool z_inbound = (pose.position.z >= 0.3 && pose.position.z <= 1.00);

  return x_inbound && y_inbound && z_inbound;
}

void Arm::MoveTargetPose(geometry_msgs::Pose target_pose, bool execute) {
  ROS_INFO("Moving to Target Pose Point");
  PrintCurrentPose(target_pose);

  if (!ValidateTargetPose(target_pose)) {
    ROS_WARN("Pose Target Exceeds Bounds");
    return;
  }

  move_group_.setPoseTarget(target_pose);
  success_ = (move_group_.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success_) {
    ROS_INFO("Plan unsuccesful...");
    return;
  }

  if (execute) {
    ROS_INFO("Attempting to execute ...");
    move_group_.execute(plan_);
    ROS_INFO("Finished Executing");
  }
}

void Arm::MoveTargetJoint(std::vector<double> target_joint, bool execute) {
  ROS_INFO("Moving to Target Joint Points:");
  PrintCurrentJointValues(target_joint);
  move_group_.setJointValueTarget(target_joint);
  success_ = (move_group_.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success_) {
    ROS_INFO("Plan unsuccesful...");
    return;
  }

  if (execute) {
    ROS_INFO("Attempting to execute ...");
    move_group_.execute(plan_);
    ROS_INFO("Finished Executing");
  }
}

// void Arm::moveCartesianPath(double jump_threshold, double eef_step, bool step, bool execute) {
//     //move robot according to cartesian path
//     if (waypoints.size() == 0) {
//         ROS_INFO("No waypoints found");
//         return;
//     }
//
//     moveit_msgs::RobotTrajectory trajectory;
//     //compute the cartesian path
//     double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//     ROS_INFO("Visualizing cartesian path (%.2f%% acheived)", fraction * 100.0);
//
//     plan_.trajectory_ = trajectory;
//
//     if (execute == true) {
//         move_group_.execute(plan_);
//         ROS_INFO("Cartesian path exexuted");
//     }
//     waypoints.clear();
//     set_waypoints = false;
//     ROS_INFO("Waypoints cleared");
// }

void Arm::executeCartesianPath(std::vector<geometry_msgs::Pose> waypoints) {
    double eef_step = 0.01;
    double jump_threshold = 0.0;
    move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    plan_.trajectory_ = trajectory;
    move_group_.execute(plan_);
}

void Arm::MoveTargetPoseCon(geometry_msgs::Pose target_pose, bool execute, double plan_time) {
    //move to a target pose with constraint
    if (!orient_constraint_ && !joint_constraint_) {
        ROS_INFO("No Constraints Set");
        return;
    }

    // move_group_.setPathConstraints(path_constraints);
    move_group_.setPlanningTime(plan_time);

    if (orient_constraint_ == true) {
        //use orientation constraint as the target orientation, otherwise there will be error
        ROS_INFO_NAMED("move_panda", "Orientaiton constraint will be set as target orientation");
        target_pose.orientation.w = ocm_.orientation.w;
        target_pose.orientation.x = ocm_.orientation.x;
        target_pose.orientation.y = ocm_.orientation.y;
        target_pose.orientation.z = ocm_.orientation.z;
    }

    MoveTargetPose(target_pose, execute);
}

}
