// Node to stop motor on RPLidar sensor
// Created so the motor can be stopped and started from a launch file

// Author: Steven Daniluk
// Date Created: 02/07/2016
// Last Modified: 02/07/2016

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "stop_rplidar_motor_client");
  ros::NodeHandle nh;

  // In order to call the service, must loop through waiting for RPlidar to start
  
  ros::Rate loop_rate(1);
  int fail_count = 0;
  
  while(ros::ok()) {
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("stop_motor");
    std_srvs::Empty srv;
    bool success = client.call(srv);
    
    if (success) {
      ROS_INFO_STREAM("RPlidar motor stopped");
      return 0;
    }else if (fail_count < 10) {
      ROS_INFO_STREAM("Waiting for RPLidar to start");
    }else {
      ROS_ERROR_STREAM("Failed to call stop_motor service");
      return 1;
    }// end success
    
    ++fail_count;
    loop_rate.sleep();
  
  }// end while
  

}// end main


