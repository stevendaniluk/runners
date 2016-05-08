// Runners SF Development script

// For performing laps around the Sanford Flemming building while recording data

#include <ros/ros.h>
#include <ros/package.h>
#include <runner.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/SensorState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <iomanip>
#include <fstream>

//---------------------------------------------------------------------------

void write_data_to_file (std::ofstream &data_file, double &start_time, Runner &runner) {
    ros::Time time = ros::Time::now();
    
    // How long we have been running
    double secs = time.toSec() - start_time;
    // Get the hours
    int hours = floor(secs/3600);
    // Get the minutes
    int minutes = floor(secs/60);
    minutes = minutes % 60;
    // Get the seconds
    secs = secs - minutes*60 - hours*3600;
    
    // Output data to file
    data_file << "\n";
    data_file << std::fixed << std::setprecision(2);
    data_file << std::setfill('0') << std::setw(2) << hours << ":" ;
    data_file << std::setfill('0') << std::setw(2) << minutes << ":";
    data_file << std::setfill('0') << std::setw(5) << secs << "\t";
    data_file << std::fixed << std::setprecision(3);
    data_file << std::setfill('0') << std::setw(6) << runner.pos_x_ << "\t\t";
    data_file << std::setfill('0') << std::setw(6) << runner.pos_y_ << "\t\t";
    data_file << runner.vel_x_ << "\t\t";
    data_file << runner.vel_theta_ << "\t\t";
    data_file << std::fixed << std::setprecision(2);
    data_file << std::setfill('0') << std::setw(5) << runner.batt_voltage_ << "\t\t";
}// end write_data

//---------------------------------------------------------------------------

int main(int argc, char **argv) {
  
  // Check input arguments
  if (argc == 1) {
    ROS_WARN("No output data filename given. Will not record data.");
  }else if (argc != 2) {
    ROS_ERROR("Improper input arguments. Accepts no inputs, or a filename");
  }// end if
  
  // Initialize
  ros::init(argc, argv, "lap_manager");
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
  
  // Create the filename from the input
  std::string output_filename = ros::package::getPath("runners") + "/testing_data/"
                                + argv[1] + ".txt";
  
  // Create our output file stream object
  std::ofstream data_file (output_filename.c_str());
  if (!data_file.is_open()) {
    ROS_ERROR("Unable to open data file for writing.");
  }// end if
  
  // Put titles in the data file
  data_file << "Time \t\tPos X \t\tPos Y \t\tV Lin \t\tV Ang \t\tBatt V";

  // Create our Runner
  Runner runner;
  
  // Useful variables
  ros::Rate fast_loop_rate(5);          // For while we are running
  ros::Rate slow_loop_rate(0.2);        // For while we are docked
  ros::Time time = ros::Time::now();    // For occasionally outputting data
  double start_time = time.toSec();     // Start time for data file
  double delta_x;			// Relative x position from dock
  double delta_y;			// Relative y position from dock
  
  // Make fake goal
  runner.setSimpleGoal(0.0, 0.0, 0.0);
  // Send out navigation goal so it stays running
  runner.sendSimpleGoal();
  
  while (ros::ok()) {
    runner.update();
    
    if (!runner.docked_) {
      // Write all our data to the text file
      write_data_to_file(data_file, start_time, runner);

      // Occasionally output our position
      if (ros::Time::now() > (time + ros::Duration(5))) {
        ROS_INFO("Position: X=%.2f, Y=%.2f", runner.pos_x_, runner.pos_y_);
        ROS_INFO("Battery Voltage: %.2f V\n", runner.batt_voltage_);
        time = ros::Time::now();
      }// end if
      
      // If we are low on battery, we'll need to dock and charge
      if (runner.batt_low_) {
        delta_x = std::abs(runner.pos_x_ - dock_x);
        delta_y = std::abs(runner.pos_y_ - dock_y);
        
        // Only start docking once we are close enough
        if (delta_x < dock_proximity && delta_y < dock_proximity) {
          ROS_INFO("Going to try and dock");
          runner.setActionGoal(dock_x, dock_y, dock_yaw);
          runner.sendActionGoal();
        }// end if
      }// end if
      
      fast_loop_rate.sleep();
      
    }else {
      // Wait until we have a full charge
      if (runner.full_charge_) {
        // Backup from the dock
        runner.undock();
        
        // Make fake goal
        runner.setSimpleGoal(0.0, 0.0, 0.0);
        // Send out navigation goal so it stays running
        runner.sendSimpleGoal();
      }else {
        // Occasionally output our battery level
        if (ros::Time::now() > (time + ros::Duration(60))) {
          ROS_INFO("Battery Voltage: %.2f V", runner.batt_voltage_);
          time = ros::Time::now();
        }// end if
      }// end if
      
      slow_loop_rate.sleep();
      
    }// end docked if
    
  }// end while
  
  // Close the data file
  data_file.close();

  return 0;
}// end main
