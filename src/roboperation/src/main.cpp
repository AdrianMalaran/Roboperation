#include <ros/ros.h>
#include <ros/console.h>
#include "roboperation/ArmControl.h"
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{
  ROS_INFO("Initalizing Main Loop for Arm Control!");
  ros::init(argc, argv, "arm_control");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Starting Arm...");
  panda::Arm arm;

  // while (ros::ok())
  // {
  //   ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  // }

  // ros::spin();
  return 0;
}
