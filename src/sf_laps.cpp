// Runners SF Development script

// For performing laps around the Sanford Flemming building while recording data

#include <ros/ros.h>
#include <runner.h>
#include <lap_manager.h>

//---------------------------------------------------------------------------

int main(int argc, char **argv) {
  
  bool saving_data;
  
  // Check input arguments
  if (argc == 1) {
    ROS_WARN("No output data filename given. Will not record data.");
    saving_data = false;
  }else if (argc != 2) {
    ROS_ERROR("Improper input arguments. Accepts no inputs, or a filename");
    return 1;
  }else {
    saving_data = true;
  }// end if
  
  // Initialize
  ros::init(argc, argv, "lap_manager");
  
  // Create our LapManager and Runner objects
  Runner runner;
  LapManager lap_manager;
  
  if (saving_data) {
    lap_manager.createDataFile(argv[1]);
  }// end if
  
  // Useful variables
  ros::Rate fast_loop_rate(5);          	// For while we are running
  ros::Rate slow_loop_rate(0.2);        	// For while we are docked
  ros::Time data_time = ros::Time::now();    	// For occasionally outputting data
  ros::Time goal_time = ros::Time::now();    	// For occasionally sending a goal
  
  ROS_INFO("Beginning test.");
  
  // Make fake goal
  runner.setSimpleGoal(0.0, 0.0, 0.0);
  
  while (ros::ok()) {
    runner.update();
    
    runner.docked_ = false;
    
    if (!runner.docked_) {
      if (saving_data) {
        // Write all our data to the text file
        lap_manager.writeDataToFile(runner);
      }// end if
      
      // Occasionally output our position
      if (ros::Time::now() > (goal_time + ros::Duration(2))) {
	runner.sendSimpleGoal();
        goal_time = ros::Time::now();
      }// end if
      
      // Occasionally output our position
      if (ros::Time::now() > (data_time + ros::Duration(5))) {
        ROS_INFO("Position: X=%.2f, Y=%.2f", runner.pos_x_, runner.pos_y_);
        ROS_INFO("Battery Voltage: %.2f V\n", runner.batt_voltage_);
        data_time = ros::Time::now();
      }// end if
      
      if (runner.batt_low_) {
        // Monitor proximity to dock, and dock when close
        lap_manager.dockWhenClose(runner);
      }// end if
      
      fast_loop_rate.sleep();
      
    }else {
      // Wait until we have a full charge
      if (runner.full_charge_) {
        // Backup from the dock, and start navigating
        runner.undock();
        runner.sendSimpleGoal();
      }else {
        // Occasionally output our battery level
        if (ros::Time::now() > (data_time + ros::Duration(60))) {
          ROS_INFO("Battery Voltage: %.2f V", runner.batt_voltage_);
          data_time = ros::Time::now();
        }// end if
      }// end if
      
      slow_loop_rate.sleep();
      
    }// end docked if
  }// end while

  return 0;
}// end main
