// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <franka/gripper_state.h>
#include <roboperation_gripper/roboperation_gripper.h>

namespace {

template <typename T_action, typename T_goal, typename T_result>
void handleErrors(actionlib::SimpleActionServer<T_action>* server,
                  std::function<bool(const T_goal&)> handler,
                  const T_goal& goal) {
  T_result result;
  try {
    result.success = handler(goal);
    server->setSucceeded(result);
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM("" << ex.what());
    result.success = false;
    result.error = ex.what();
    server->setAborted(result);
  }
}

}  // anonymous namespace

using actionlib::SimpleActionServer;
using control_msgs::GripperCommandAction;
using franka_gripper::GraspAction;
using franka_gripper::GraspEpsilon;
using franka_gripper::GraspGoalConstPtr;
using franka_gripper::GraspResult;
using franka_gripper::HomingAction;
using franka_gripper::HomingGoalConstPtr;
using franka_gripper::HomingResult;
using franka_gripper::MoveAction;
using franka_gripper::MoveGoalConstPtr;
using franka_gripper::MoveResult;
using franka_gripper::StopAction;
using franka_gripper::StopGoalConstPtr;
using franka_gripper::StopResult;
using franka_gripper::grasp;
using franka_gripper::gripperCommandExecuteCallback;
using franka_gripper::homing;
using franka_gripper::move;
using franka_gripper::stop;
using franka_gripper::updateGripperState;


enum GrippingActionState {
  STOP=0,
  CLOSE=1,
  OPEN=2
};

GrippingActionState gripper_action_state = STOP;

void StateCallback(const std_msgs::String::ConstPtr &msg) {
    // std_msgs::String state = msg->data.c_str();
    ROS_INFO("Desired State: %s", msg->data.c_str());

    gripper_action_state = GrippingActionState::CLOSE;
    if (msg->data == "CLOSE") {
      gripper_action_state = GrippingActionState::CLOSE;
    } else if (msg->data == "OPEN") {
      gripper_action_state = GrippingActionState::OPEN;
    } else {
      gripper_action_state = GrippingActionState::STOP;
    }
}

int main(int argc, char** argv) {
  ROS_WARN("Starting Gripper Node!");
  ros::init(argc, argv, "franka_gripper_node");
  ros::NodeHandle node_handle("~");
  std::string robot_ip;
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  double default_speed(0.1);
  if (node_handle.getParam("default_speed", default_speed)) {
    ROS_INFO_STREAM("franka_gripper_node: Found default_speed " << default_speed);
  }

  GraspEpsilon default_grasp_epsilon;
  default_grasp_epsilon.inner = 0.005;
  default_grasp_epsilon.outer = 0.005;
  std::map<std::string, double> epsilon_map;
  if (node_handle.getParam("default_grasp_epsilon", epsilon_map)) {
    ROS_INFO_STREAM("franka_gripper_node: Found default_grasp_epsilon "
                    << "inner: " << epsilon_map["inner"] << ", outer: " << epsilon_map["outer"]);
    default_grasp_epsilon.inner = epsilon_map["inner"];
    default_grasp_epsilon.outer = epsilon_map["outer"];
  }

  franka::Gripper gripper(robot_ip);

  auto homing_handler = [&gripper](auto&& goal) { return homing(gripper, goal); };
  auto stop_handler = [&gripper](auto&& goal) { return stop(gripper, goal); };
  auto grasp_handler = [&gripper](auto&& goal) { return grasp(gripper, goal); };
  auto move_handler = [&gripper](auto&& goal) { return move(gripper, goal); };

  SimpleActionServer<HomingAction> homing_action_server(
      node_handle, "homing",
      [=, &homing_action_server](auto&& goal) {
        return handleErrors<franka_gripper::HomingAction, franka_gripper::HomingGoalConstPtr,
                            franka_gripper::HomingResult>(&homing_action_server, homing_handler,
                                                          goal);
      },
      false);

  SimpleActionServer<StopAction> stop_action_server(
      node_handle, "stop",
      [=, &stop_action_server](auto&& goal) {
        return handleErrors<franka_gripper::StopAction, franka_gripper::StopGoalConstPtr,
                            franka_gripper::StopResult>(&stop_action_server, stop_handler, goal);
      },
      false);

  SimpleActionServer<MoveAction> move_action_server(
      node_handle, "move",
      [=, &move_action_server](auto&& goal) {
        return handleErrors<franka_gripper::MoveAction, franka_gripper::MoveGoalConstPtr,
                            franka_gripper::MoveResult>(&move_action_server, move_handler, goal);
      },
      false);

  SimpleActionServer<GraspAction> grasp_action_server(
      node_handle, "grasp",
      [=, &grasp_action_server](auto&& goal) {
        return handleErrors<franka_gripper::GraspAction, franka_gripper::GraspGoalConstPtr,
                            franka_gripper::GraspResult>(&grasp_action_server, grasp_handler, goal);
      },
      false);

  SimpleActionServer<GripperCommandAction> gripper_command_action_server(
      node_handle, "gripper_action",
      [=, &gripper, &gripper_command_action_server](auto&& goal) {
        return gripperCommandExecuteCallback(gripper, default_grasp_epsilon, default_speed,
                                             &gripper_command_action_server, goal);
      },
      false);

  ROS_WARN("Starting Actions");
  homing_action_server.start();
  stop_action_server.start();
  move_action_server.start();
  grasp_action_server.start();
  gripper_command_action_server.start();

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("franka_gripper_node: Could not find parameter publish_rate. Defaulting to "
                    << publish_rate);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("franka_gripper_node: Could not parse joint_names!");
    return -1;
  }
  if (joint_names.size() != 2) {
    ROS_ERROR("franka_gripper_node: Got wrong number of joint_names!");
    return -1;
  }

  franka::GripperState gripper_state;
  std::mutex gripper_state_mutex;
  std::thread read_thread([&gripper_state, &gripper, &gripper_state_mutex]() {
    ros::Rate read_rate(30);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> _(gripper_state_mutex);
        updateGripperState(gripper, &gripper_state);
      }
      read_rate.sleep();
    }
  });

  ros::Publisher gripper_state_publisher =
      node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Subscriber close_gripper_subscriber = node_handle.subscribe("chatter", 1, &StateCallback);


  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Start moving Gripper
  gripper.homing();
  gripper_state = gripper.readOnce();
  double max_width = gripper_state.max_width;
  ROS_INFO("Max Grasping Width: Width:%f, MaxWidth:%f", gripper_state.width, gripper_state.max_width);
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
  gripper.stop();

  ROS_WARN("Starting Gripper ...");
  ros::Rate rate(publish_rate);
  ros::Rate gripper_release_rate(10);

  bool has_grasped = false;

  while (ros::ok()) {
    gripper_state = gripper.readOnce();

    double gripper_speed = 0.5;
    double gripper_position_step = 0.008;

    if (gripper_action_state == GrippingActionState::CLOSE) {
      ROS_INFO("Set GripperActionState: CLOSE");
    } else if (gripper_action_state == GrippingActionState::OPEN) {
      ROS_INFO("Set GripperActionState: OPEN");
    }

    if (gripper_action_state == GrippingActionState::STOP) {
      gripper.stop();
    } else {
      if (gripper_action_state == GrippingActionState::CLOSE && !has_grasped) {
        ROS_INFO("State Grasping Gripper");
        if (!gripper.grasp(0.057, 0.1, 45)) {
          ROS_WARN("No Object Grasped");
          gripper_action_state = GrippingActionState::OPEN;
        } else {
          // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));
          while(gripper_action_state == GrippingActionState::CLOSE) {
            ROS_WARN("Gripped Object - Waiting for Release ...");
            gripper_release_rate.sleep();
          }
          ROS_WARN("--------------FINISHED!");
          gripper.stop();
        }

        ROS_INFO("Grasping the object (%u)", gripper_state.is_grasped);
        has_grasped = gripper_state.is_grasped;
      }
      else {
        ROS_INFO("Opening Gripper");
        gripper.move(max_width, gripper_speed);
        gripper_action_state = GrippingActionState::STOP;
      }
    }

    rate.sleep();
  }
  read_thread.join();
  return 0;
}
