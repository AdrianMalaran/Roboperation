#include "roboperation/ArmControl.h"

namespace panda {

Arm::Arm() : move_group_(planning_group_) {
  ROS_INFO("Initializing Arm Control.");

  input_arm_state_ = nh_.subscribe("input_state", 1, &Arm::PoseListenerCallback, this);
  string_subscriber_ = nh_.subscribe("chatter", 1, &Arm::StringMessageCallback, this);

  // SUBSCRIBER EXAMPLE
  // planning_mode_sub_(nh_.subscribe("planning_mode",
  //                                      10,
  //                                      &LocalPlanner::PlanningModeCallback,
  //                                      this)),
  // ADVERTISER EXAMPLE
  // lqr_path_pub_(
  //         nh_.advertise<caravel_msgs::PathPointArray>("path", 10, false)),

  // moveit::planning_interface::MoveGroupInterface move_group_(planning_group_); //move_group_ object storing joint and link info

  home_pose_ = move_group_.getCurrentPose().pose;
  home_joint_ = move_group_.getCurrentJointValues();

  ROS_INFO("Home Pose: Pos(%.2f,%.2f,%.2f)", home_pose_.position.x, home_pose_.position.y, home_pose_.position.z);
  // ROS_INFO("Home Joint:Pos(%.2f,%.2f,%.2f)", home_joint_.position.x, home_joint_.position.y, home_joint_.position.z);
}

void Arm::StringMessageCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received String Message");
}

void Arm::PoseListenerCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  ROS_INFO("Received Desired Pose.");
}

void Arm::MoveArm() {
  ROS_INFO("Executing Arm Command");
}

void Arm::MoveTargetPose(geometry_msgs::Pose target_pose, bool execute) {
  ROS_INFO("Moving to Target Pose Point(%f,%f,%f)", target_pose.position.x, target_pose.position.y, target_pose.position.z);
  move_group_.setPoseTarget(target_pose);
  success_ = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success_) {
    ROS_INFO("Plan unsuccesful...");
    return;
  }

  if (execute) {
    move_group_.execute(plan);
    ROS_INFO("Finished Executing");
  }
}

void Arm::MoveTargetPoseCon(geometry_msgs::Pose target_pose, bool execute, double plan_time) {
    //move to a target pose with constraint
    if (orient_constraint_ == false && joint_constraint_ == false) {
        ROS_INFO_NAMED("move_group_", "No constraint set");
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

void Arm::moveCartesianPath(double jump_threshold, double eef_step, bool step, bool execute)
{ //move robot according to cartesian path
    if (waypoints.size() == 0)
    {
        ROS_INFO_NAMED("move_panda", "No waypoints found");
        return;
    }

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); //compute the cartesian path
    ROS_INFO_NAMED("move_panda", "Visualizing cartesian path (%.2f%% acheived)", fraction * 100.0);

    //scale the trajectory according to the velocity factor (need more work at this section)
    /*robot_model_loader::RobotModelLoader robot_model_loader(move_group_.ROBOT_DESCRIPTION);
    robot_model::RobotModelConstPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(robot_model);
    trajectory_processing::IterativeParabolicTimeParameterization scale_time;
    robot_trajectory::RobotTrajectory robot_traj(robot_model, group);
    robot_traj.getRobotTrajectoryMsg(trajectory);
    bool scale_traj = scale_time.computeTimeStamps(robot_traj, vel_factor); //scale the trajectory time steps as per the velocity factor
    if (scale_traj == true)
    {
        ROS_INFO_NAMED("move_panda", "Trajectory scaled according to velocity factor %.2f", vel_factor);
    }
    else
    {
        ROS_INFO_NAMED("move_panda", "Trajecotry scaling failed. Velocity factor = 1.0 is used");
    }
    robot_traj.setRobotTrajectoryMsg(robot_state, trajectory);*/

    plan.trajectory_ = trajectory;
    if (step == true)
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    }

    if (execute == true)
    {
        move_group_.execute(plan);
        ROS_INFO_NAMED("move_panda", "Cartesian path exexuted");
        // if (step == true)
        // {
        //     visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        // }
    }
    waypoints.clear();
    set_waypoints = false;
    ROS_INFO_NAMED("move_panda", "Waypoints cleared");
}



}
