// Runners SF Development script

// For performing laps around the Sanford Flemming building while recording data

#include "ros/ros.h"
#include "runner.h"
#include "std_msgs/String.h"
#include "kobuki_msgs/SensorState.h"

//---------------------------------------------------------------------------

int main(int argc, char **argv) {
  // Initialize
  ros::init(argc, argv, "test");
  
  // Make our node handle
  ros::NodeHandle nh;
  
  // Create our Runner
  Runner runner;
  
  // Make fake goal
  runner.setCurrentGoal(0.0, 0.0, 0.0, 1.0);
  
  // Start navigating
  runner.sendGoal(runner.current_goal_);
  
  // Lets loop at 1 Hz
  ros::Rate loop_rate(0.5);
  
  while (ros::ok()) {
    runner.update();
    
    ROS_INFO("Voltage: %.2f", runner.batt_voltage_);
    
    loop_rate.sleep();
  }// end while
  

  return 0;
}// end main
