// Class definition for Lap Manager

#ifndef LAPMANAGER_H
#define LAPMANAGER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <runner.h>
#include <std_msgs/String.h>
#include <iomanip>
#include <fstream>
#include <iostream>

class LapManager {
  public:
    // Constructor
    LapManager();
    
    // Create data file
    void createDataFile (std::string filename);
    
    // For writing data to text file
    void writeDataToFile (Runner &runner);
    
    // Monitor dock proximity, and dock when lose enough
    void dockWhenClose(Runner &runner);
  
  private:
    // Create nodehandle(s)
    ros::NodeHandle param_nh_;
    
    // Variables to load parameters into
    double dock_x_;
    double dock_y_;
    double dock_yaw_;
    double dock_proximity_;
    
    // Data file variables
    ros::Time start_time_;		// For keeping time
    std::ofstream data_file_;		// File to save data to
    
};// end class

#endif
