#include <ros/ros.h>
#include <runner.h>

//----------------------------------

int main(int argc, char **argv) {
  // Initialize
  ros::init(argc, argv, "listener");
  
  // Make our node handle
  ros::NodeHandle param_nh("/ExecutivePlanner");
  
  // Variables to lead parameters into
  double dock_x;
  double dock_y;
  double dock_yaw;
  double dock_proximity;
     
  // Get parameters
  param_nh.param("dock_x", dock_x, 0.00);
  param_nh.param("dock_y", dock_y, 0.00);
  param_nh.param("dock_yaw", dock_yaw, 0.00);
  param_nh.param("dock_proximity", dock_proximity, 1.00);
  
  // Create the runner
  Runner runner;
  
  // Dock
  ROS_INFO("Going to try and dock");
  runner.setActionGoal(dock_x, dock_y, dock_yaw);
  runner.sendActionGoal();
  
  runner.dock();
  
  ros::Duration(3).sleep();
  
  // Backup from the dock
  runner.undock();


  return 0;
}// end main
