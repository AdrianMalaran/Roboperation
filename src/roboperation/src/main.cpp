#include <ros/ros.h>
#include <ros/console.h>
#include "roboperation/ArmControl.h"

int main(int argc, char **argv)
{
  ROS_INFO("Initalizing Arm Controller");

  ros::init(argc, argv, "arm_control");
  panda::Arm arm;

  ros::spin();

  return 0;
}
