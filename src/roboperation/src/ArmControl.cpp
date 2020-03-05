#include "roboperation/ArmControl.h"

/*

Plan For Today:
[] Have a collision out-of-bounds
[] Run Arduino as a ros node on linux on docker
[] Trigger a move to start position
[x] Create a test script that
[x] Highjack the /equilibrium_pose
[x] Set boundary constraints on the desired position
[x] Set Warning for high force action
[] Need a reset to home pose script
[] Maybe try controlling the gripper

-// roslaunch panda_moveit_config demo.launch

INSTRUCTIONS TO RUN:
  Pre-instructions:
    1) Power on robot
    2) Unlock Joints https://franka2.robohub.eng.uwaterloo.ca/desk/
    3) Start Terminal
      Each Terminal:
        start_docker
        source /opt/ros/kinetic/setup.bash
        cd ~/Roboperation
        source devel/setup.bash
      Terminal 1:
        roslaunch roboperation roboperation_controller.launch
      Terminal 2:
        cd ~/Roboperation/src/roboperation &&

INSTRUCTIONS TO SHUT DOWN:
1) Log out of Google
2) Log out of Github
3) Unplug Arduino/Chargers
4) Plug in USB Camera 1
5) Lock Joints
6) E-STOP/Power Off


LAUNCH CONTROLLER:
$$$$ roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=franka2

EXAMPLE Message:
$$$$ rostopic echo /equilibrium_pose
server_id: "/interactive_marker"
seq_num: 3930
type: 1
markers: []
poses:
  -
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs:         0
      frame_id: "panda_link0"
    pose:
      position:
        x: 0.403116226196
        y: 0.0359357595444
        z: 0.385318487883
      orientation:
        x: 0.988982498646
        y: -0.00681050354615
        z: 0.0274160169065
        w: 0.145312517881
    name: "equilibrium_pose"
erases: []



POSSIBLE TOPICS:

r/clicked_point
/dynamic_reconfigure_compliance_param_node/parameter_descriptions
/dynamic_reconfigure_compliance_param_node/parameter_updates
/equilibrium_pose
/franka_control/error_recovery/cancel
/franka_control/error_recovery/feedback
/franka_control/error_recovery/goal
/franka_control/error_recovery/result
/franka_control/error_recovery/status
/franka_gripper/grasp/cancel
/franka_gripper/grasp/feedback
/franka_gripper/grasp/goal
/franka_gripper/grasp/result
/franka_gripper/grasp/status
/franka_gripper/gripper_action/cancel
/franka_gripper/gripper_action/feedback
/franka_gripper/gripper_action/goal
/franka_gripper/gripper_action/result
/franka_gripper/gripper_action/status
/franka_gripper/homing/cancel
/franka_gripper/homing/feedback
/franka_gripper/homing/goal
/franka_gripper/homing/result
/franka_gripper/homing/status
/franka_gripper/joint_states
/franka_gripper/move/cancel
/franka_gripper/move/feedback
/franka_gripper/move/goal
/franka_gripper/move/result
/franka_gripper/move/status
/franka_gripper/stop/cancel
/franka_gripper/stop/feedback
/franka_gripper/stop/goal
/franka_gripper/stop/result
/franka_gripper/stop/status
/franka_state_controller/F_ext
/franka_state_controller/franka_states
/franka_state_controller/joint_states
/franka_state_controller/joint_states_desired
/initialpose
/input_state
/joint_states
/joint_states_desired
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static


*/

namespace panda {

Arm::Arm()
: move_group_(planning_group_)
, visual_tools(move_group_.getLinkNames()[0]) {
    ROS_INFO("Initializing Arm Control.");

    SetPubsAndSubs();
    Reset();
    // Set the max velocity scaling factor
    setMaxVelScalingFactor(1.0);

    home_pose_ = move_group_.getCurrentPose().pose;
    home_joint_ = move_group_.getCurrentJointValues();

    PrintPose(move_group_.getCurrentPose().pose);

    // TestMovements();
    Loop();
}

void Arm::Loop() {
    while (ros::ok()) {
        // PrintPose(move_group_.getCurrentPose().pose);
        // PrintJointValues(move_group_.getCurrentJointValues());
        ExecuteBufferedActions();
        // MoveTargetPose(most_recent_goal_state_, true);
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        ros::Duration(0.0).sleep();
    }
}

void Arm::SetPubsAndSubs() {
  // Subscribers
  input_arm_state_ = nh_.subscribe("input_state", 1, &Arm::PoseListenerCallback, this);
  string_subscriber_ = nh_.subscribe("chatter", 1, &Arm::StringMessageCallback, this);
  arm_control_input_ = nh_.subscribe("arm_control_input", 1, &Arm::MoveArmRealTimeCallback, this);

  // Publishers
  robot_arm_error_publisher_ = nh_.advertise<std_msgs::String>("/error", 1, EXECUTE_MOTION);
}

void Arm::Reset() {
  ROS_WARN("Resetting all variables");
  // Robot Initializations
  visual_tools.deleteAllMarkers();
  action_buffer_ = queue<roboperation::ArmStatePose>();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //TODO: Remove if not necessary
  joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(planning_group_);
}

void Arm::PrintPose(geometry_msgs::Pose pose_values) {
    ROS_INFO("  Point: Pos(%.2f, %.2f, %.2f)", pose_values.position.x, pose_values.position.y, pose_values.position.z);
    ROS_INFO("  Orien: Ori(%.4f,%.4f, %.4f, %.4f)", pose_values.orientation.w, pose_values.orientation.x, pose_values.orientation.y, pose_values.orientation.z);
}

void Arm::PrintJointValues(std::vector<double> joint_values) {
    std::string joints;
    int current_joint = 0;
    for (double joint : joint_values) {
        joints += "J_" + to_string(current_joint) + ": " + to_string(joint) + ", ";
        current_joint += 1;
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
  // If the action buffer is non-empty, execute the next action
  if (!action_buffer_.empty()) {
    ROS_INFO("Executing next buffered pose");
    geometry_msgs::Pose next_pose;
    next_pose = action_buffer_.front().desired_pose;
    action_buffer_.pop();
    MoveTargetPose(next_pose, EXECUTE_MOTION);
  } else {
    ROS_INFO("No goal pose buffered - waiting ...");
  }
}

void Arm::MoveArmRealTimeCallback(const roboperation::ArmStatePose::ConstPtr &msg) {
    ROS_INFO("Got Arm State Pose");

    geometry_msgs::Pose target_pose;
    target_pose.position = msg->desired_pose.position;
    target_pose.orientation = msg->desired_pose.orientation;
    // TODO: Gotta have a if arm state is there then
    if (EXECUTE_MOTION) {
      ROS_WARN("  Active - Moving Arm..");
      PrintPose(target_pose);
      roboperation::ArmStatePose target_state;
      target_state.desired_pose = target_pose;
      action_buffer_.push(target_state);
      most_recent_goal_state_ = target_pose;
      // ExecuteBufferedActions();
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

      bool x_inbound = (pose.position.x >= x_bounds[0] && pose.position.x <= x_bounds[1]);
      bool y_inbound = (pose.position.y >= y_bounds[0] && pose.position.y <= y_bounds[1]);
      bool z_inbound = (pose.position.z >= z_bounds[0] && pose.position.z <= z_bounds[1]);

      // Validate the cartesian path to be travelled ensure its lower than a specific threshold
      double relative_distance = 0.0;
      if (relative_distance <= small_distance_threshold) {
        // TODO: Todo
      }
      return x_inbound && y_inbound && z_inbound;
}

bool ValidateTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan) {
  bool valid_trajectory = true;
  int size = plan.trajectory_.joint_trajectory.points.size();
  ROS_INFO("JointTrajectories: %i", size);

  if (plan.trajectory_.joint_trajectory.points.size() > 10) { //TODO: Revise this
    ROS_WARN("Trajectory Point Size %i", size);
    valid_trajectory = false;
  }

  return valid_trajectory;
}

void Arm::VisualizeTrajectory() {
      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.75;
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(plan_.trajectory_, joint_model_group_);
      visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

void Arm::MoveTargetPose(geometry_msgs::Pose target_pose, bool execute) {
      ROS_INFO("Moving to Target Pose Point");
      PrintPose(target_pose);

      if (!ValidateTargetPose(target_pose)) {
            ROS_WARN("Out of Bounds (%.2f, %.2f [], %.2f)",
            target_pose.position.x, target_pose.position.y, target_pose.position.z);
            return;
      }

      move_group_.setPoseTarget(target_pose);
      success_ = (move_group_.plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (!success_) {
        ROS_WARN("Plan unsuccesful...");
        return;
      } else if (!ValidateTrajectory(plan_)) {
        ROS_WARN("Invalid Trajectory - It might be too long ?");
      }

      VisualizeTrajectory();

      if (execute) {
            ROS_WARN("Attempting to execute ...");
            move_group_.execute(plan_);
            ROS_INFO("Finished Executing");
      }
      // Set plan to empty to ensure we don't execute the same plan twice
      plan_.trajectory_.joint_trajectory.points = {};
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

// void setDefaultBehavior(franka::Robot& robot) {
//   robot.setCollisionBehavior(
//       {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//       {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
//       {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
//       {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
//   robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
//   robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
// }
//
// void ImpedanceController() {
//
//   // Compliance parameters
//   const double translational_stiffness{150.0};
//   const double rotational_stiffness{10.0};
//   Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
//   stiffness.setZero();
//   stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
//   stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
//   damping.setZero();
//   damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
//                                      Eigen::MatrixXd::Identity(3, 3);
//   damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
//                                          Eigen::MatrixXd::Identity(3, 3);
//
//   try {
//     // connect to robot
//     franka::Robot robot("franka2");
//     setDefaultBehavior(robot);
//     // load the kinematics and dynamics model
//     franka::Model model = robot.loadModel();
//
//     franka::RobotState initial_state = robot.readOnce();
//
//     // equilibrium point is the initial position
//     Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
//     Eigen::Vector3d position_d(initial_transform.translation());
//     Eigen::Quaterniond orientation_d(initial_transform.linear());
//
//     // set collision behavior
//     robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
//
//     // define callback for the torque control loop
//     std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
//         impedance_control_callback = [&](const franka::RobotState& robot_state,
//                                          franka::Duration /*duration*/) -> franka::Torques {
//       // get state variables
//       std::array<double, 7> coriolis_array = model.coriolis(robot_state);
//       std::array<double, 42> jacobian_array =
//           model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
//
//       // convert to Eigen
//       Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
//       Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
//       Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
//       Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
//       Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
//       Eigen::Vector3d position(transform.translation());
//       Eigen::Quaterniond orientation(transform.linear());
//
//       // compute error to desired equilibrium pose
//       // position error
//       Eigen::Matrix<double, 6, 1> error;
//       error.head(3) << position - position_d;
//
//       // orientation error
//       // "difference" quaternion
//       if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
//         orientation.coeffs() << -orientation.coeffs();
//       }
//       // "difference" quaternion
//       Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
//       error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
//       // Transform to base frame
//       error.tail(3) << -transform.linear() * error.tail(3);
//
//       // compute control
//       Eigen::VectorXd tau_task(7), tau_d(7);
//
//       // Spring damper system with damping ratio=1
//       tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
//       tau_d << tau_task + coriolis;
//
//       std::array<double, 7> tau_d_array{};
//       Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
//       return tau_d_array;
//     };
//
//     // start real-time control loop
//     std::cout << "WARNING: Collision thresholds are set to high values. "
//               << "Make sure you have the user stop at hand!" << std::endl
//               << "After starting try to push the robot and see how it reacts." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(impedance_control_callback);
//
//   } catch (const franka::Exception& ex) {
//     // print exception
//     // std::cout << ex.what() << std::endl;
//     ROS_ERROR("Exception in Cartesian Impedance Controller")
//   }
// }

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

  // ROS_INFO("Testing Waypoints:");
  // for (geometry_msgs::Pose point : test_waypoints) {
  //       PrintPose(point);
  // }




  //Execute Motion
  MoveTargetPose(test_goal_pose, true);
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
