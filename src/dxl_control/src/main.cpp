
#include <stdint.h>
#include <ros/ros.h>
#include "dxl_control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dxl_control_node");


  //SnakeRobotMaster::Initialize();
  DynamixelControl::Initialize();
  ROS_INFO("Initialized : dxl_control node");
  while(ros::ok()) {
    ros::spin();
  }
}

