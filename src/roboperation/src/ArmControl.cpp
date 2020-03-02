#include "roboperation/ArmControl.h"

/*

Plan For Today:
    []  Make a callback that is responsive to moving
[x] Make the camera work
[] Use the fake controller to see how it works out on big laptop so you can code offline
[] Have a collision out-of-bounds

-// roslaunch panda_moveit_config demo.launch

Questions to Ask:
- Is there a way to interrupt the current trajectory plan to start a new one (?)
- How do i interface with a different controller (JointGroupPositionController) to try to get it to be more real time
- Would I be able to come in later @ night possibly to robohub ?
- What does playing around with the joint stiffness do ??
*/

namespace panda {

Arm::Arm()
: move_group_(planning_group_)
, visual_tools(move_group_.getLinkNames()[0]) {
    ROS_INFO("Initializing Arm Control.");

    // Robot Initializations
    // visual_tools.loadRemoteControl();
    visual_tools.deleteAllMarkers();


    // Subscribers
    input_arm_state_ = nh_.subscribe("input_state", 1, &Arm::PoseListenerCallback, this);
    string_subscriber_ = nh_.subscribe("chatter", 1, &Arm::StringMessageCallback, this);
    arm_control_input_ = nh_.subscribe("arm_control_input", 1, &Arm::MoveArmRealTimeCallback, this);

    // Publishers
    robot_arm_error_publisher_ = nh_.advertise<std_msgs::String>("/error", 1, EXECUTE_MOTION);

    // moveit::planning_interface::MoveGroupInterface move_group_(planning_group_); //move_group_ object storing joint and link info

    static const std::string PLANNING_GROUP = "panda_arm";

    // Set the max velocity scaling factor
    setMaxVelScalingFactor(0.5);

    // moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(planning_group_);
    home_pose_ = move_group_.getCurrentPose().pose;
    home_joint_ = move_group_.getCurrentJointValues();

    // Loop();

    TestMovements();
}

void Arm::Loop() {
    while (ros::ok()) {
        // Print Current Position
        // PrintPose(move_group_.getCurrentPose().pose);
        // PrintJointValues(move_group_.getCurrentJointValues());
        ExecuteBufferedActions();
        ROS_INFO("Waiting...");
        ros::Duration(0.5).sleep();
    }
}

void Arm::PrintPose(geometry_msgs::Pose pose_values) {
    ROS_INFO("Point: Pos(%.2f, %.2f, %.2f)", pose_values.position.x, pose_values.position.y, pose_values.position.z);
}

void Arm::PrintJointValues(std::vector<double> joint_values) {
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

void Arm::ExecuteBufferedActions() {
  ROS_INFO("Executing next Buffered Pose");
  // If the action buffer is non-empty, execute the next action
  if (!action_buffer_.empty()) {
    geometry_msgs::Pose next_pose;
    next_pose = action_buffer_.front().desired_pose;
    MoveTargetPose(next_pose, EXECUTE_MOTION);
  }
}

void Arm::MoveArmRealTimeCallback(const roboperation::ArmStatePose::ConstPtr &msg) {
    ROS_INFO("Got Arm State Pose");

    geometry_msgs::Pose target_pose;
    target_pose.position = msg->desired_pose.position;
    target_pose.orientation = msg->desired_pose.orientation;
    // TODO: Gotta have a if arm state is there then
    if (msg->active_state == EXECUTE_MOTION) {
      ROS_WARN("  Active button pressed - Adding motion to buffer");
      roboperation::ArmStatePose target_state;
      target_state.desired_pose = target_pose;
      action_buffer_.push(target_state);
      ExecuteBufferedActions();
      // MoveTargetPose(target_pose, EXECUTE_MOTION);
    } else {
      ROS_INFO("  Button not pressed");
    }
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

      // Validate the cartesian path to be travelled ensure its lower than a specific threshold
      double relative_distance = 0.0;
      if (relative_distance <= small_distance_threshold) {
        // TODO: Todo
      }
      return x_inbound && y_inbound && z_inbound;
}

void Arm::VisualizeTrajectory() {
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.75;
      // visual_tools.publishText(text_pose, "Roboperation Testing", rvt::WHITE, rvt::XLARGE);
      // visual_tools.publishAxisLabeled(target_pose, "pose1");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(plan_.trajectory_, joint_model_group_);
      visual_tools.trigger();
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

void Arm::MoveTargetPose(geometry_msgs::Pose target_pose, bool execute) {
      ROS_INFO("Moving to Target Pose Point");
      PrintPose(target_pose);

      if (!ValidateTargetPose(target_pose)) {
            ROS_WARN("Pose Target Exceeds Bounds (%.2f, %.2f, %.2f) - Not executing trajectory",
            target_pose.position.x, target_pose.position.y, target_pose.position.y);
            return;
      }

      move_group_.setPoseTarget(target_pose);
      // TODO: uncomment
      success_ = (move_group_.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!success_) {
            ROS_WARN("Plan unsuccesful...");
            return;
      }

      VisualizeTrajectory();

      if (execute) {
            ROS_WARN("Attempting to execute ...");
            move_group_.execute(plan_);
            ROS_INFO("Finished Executing");
      }
}

void Arm::MoveTargetJoint(std::vector<double> target_joint, bool execute) {
      ROS_INFO("Moving to Target Joint Points:");
      PrintJointValues(target_joint);
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

void Arm::MoveArmDirectionX(double distance) {
      ROS_INFO("Moving Arm %.2f in y direction", distance);
      geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
      geometry_msgs::Pose next_pose = current_pose;

      next_pose.position.x += distance;

      MoveTargetPose(next_pose, true);
}

void Arm::MoveArmDirectionY(double distance) {
      ROS_INFO("Moving Arm %.2f in y direction", distance);
      geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
      geometry_msgs::Pose next_pose = current_pose;

      next_pose.position.y += distance;

      MoveTargetPose(next_pose, true);
}

void Arm::MoveArmDirectionZ(double distance) {
      ROS_INFO("Moving Arm %.2f in z direction", distance);
      geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
      geometry_msgs::Pose next_pose = current_pose;

      next_pose.position.z += distance;

      MoveTargetPose(next_pose, true);
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

void Arm::executeCartesianPath(std::vector<geometry_msgs::Pose> waypoints, bool execute) {
            double eef_step = 0.01;
            double jump_threshold = 0.0;
            move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            plan_.trajectory_ = trajectory;

            VisualizeTrajectory();

            if (execute) {
                  ROS_INFO("Executing plan for cartesian path");
                  move_group_.execute(plan_);
            }
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

// Libraries
double Arm::getSquaredEuclideanDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
  double distanceSquared = (p1.position.x - p2.position.x) * (p1.position.x - p2.position.x)
                  + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y)
                  + (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z);
  // distance = sqrt(distance);
  return distanceSquared;
}

void Arm::TestMovements() {
      geometry_msgs::Pose test_goal_pose;
      test_goal_pose.position = home_pose_.position;
      test_goal_pose.position.y += 0.01;
      test_goal_pose.orientation = home_pose_.orientation;

      std::vector<double> home_joint_values =
            {-1.527311, 0.275241, 2.603782, -2.058497, -0.155177, 1.864411, -2.771809};

      std::vector<double> test_goal_joints1 =
            {0.301032, 0.428689, 1.150879, -2.100787, 2.670032, 0.879070, 0.00};
      std::vector<double> test_goal_joints2 =
            {0.301032, 0.428689, 1.150879, -2.100787, 2.670032, 0.879070, 1.00};

      std::vector<geometry_msgs::Pose> test_waypoints;
      for (int i = 0; i < 5; i++) {
            geometry_msgs::Pose point;
            point.position.x = home_pose_.position.x - i*1.0/100.0;
            point.position.y = home_pose_.position.y + i*1.0/100.0;
            point.position.z = home_pose_.position.z;
            point.orientation = home_pose_.orientation;
            test_waypoints.push_back(point);
      }

      for (int i = 5; i <= 10; i++) {
            geometry_msgs::Pose point;
            point.position.x = home_pose_.position.x - i*1.0/100.0;
            point.position.y = home_pose_.position.y - i*1.0/100.0;
            point.position.z = home_pose_.position.z;
            point.orientation = home_pose_.orientation;
            test_waypoints.push_back(point);
      }

      ROS_INFO("Testing Waypoints:");
      for (geometry_msgs::Pose point : test_waypoints) {
            PrintPose(point);
      }

      //Execute Motion
      // MoveTargetPose(test_goal_pose, true);
      // MoveTargetJoint(test_goal_joints1, true);
      // MoveTargetJoint(home_joint_values, true);
      // executeCartesianPath(test_waypoints, true);

      // for (int i = 0; i < 2; i ++) {
          //   MoveArmDirectionX(-0.02); // 1 cm
      // }
      //
      // for (int i = 0; i < 2; i ++) {
          //   MoveArmDirectionY(-0.02); // 1 cm
      // }
      //
      // for (int i = 0; i < 2; i ++) {
          //   MoveArmDirectionZ(-0.02); // 1 cm
      // }

      geometry_msgs::Pose p1;
      geometry_msgs::Pose p2;
      p1.position.x = 1.0;
      p1.position.y = 1.0;
      p1.position.z = 1.0;
      // Test Function
      double dist = getSquaredEuclideanDistance(p1, p2);
      ROS_INFO("Distance Calculated: %f", dist);
}

}
