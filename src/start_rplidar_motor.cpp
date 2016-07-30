// Node to start motor on RPLidar sensor
// Created so the motor can be stopped and started from a launch file

// Author: Steven Daniluk
// Date Created: 02/07/2016
// Last Modified: 02/07/2016

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

int main(int argc, char **argv) {
  
  // Initialize
  ros::init(argc, argv, "start_rplidar_motor_client");
  ros::NodeHandle nh;

  // Call the start_motor service
  ros::ServiceClient start_client = nh.serviceClient<std_srvs::Empty>("start_motor");
  std_srvs::Empty srv;
  bool start_success = start_client.call(srv);
  
  // Check that it was successful
  if (start_success) {
    ROS_INFO_STREAM("RPlidar motor started");
  } else {
    ROS_ERROR_STREAM("Failed to call start_motor service");
  }// end success


}// end main


