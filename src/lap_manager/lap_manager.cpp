// Class definition for Lap Manager

#include <lap_manager.h>

LapManager::LapManager() : param_nh_("/runners/ExecutivePlanner") {
  
  // Get parameters
  param_nh_.param("dock_x", dock_x_, 0.00);
  param_nh_.param("dock_y", dock_y_, 0.00);
  param_nh_.param("dock_yaw", dock_yaw_, 0.00);
  param_nh_.param("dock_proximity", dock_proximity_, 1.00);
  
  // For keeping time in the data file
  start_time_ = ros::Time::now();
  
}// end constructor

//-----------------------------------------

void LapManager::createDataFile (std::string filename) {
  // Create the filename from the input
  std::string output_filename = ros::package::getPath("runners") + "/testing_data/"
                                + filename + ".txt";
  
  // Open the output file stream object
  data_file_.open(output_filename.c_str());
  if (!data_file_.is_open()) {
    ROS_ERROR("Unable to open data file for writing.");
  }// end if
  
  // Put titles in the data file
  data_file_ << "Time \t\tPos X \t\tPos Y \t\tV Lin \t\tV Ang \t\tBatt V";
}// end createDataFile

//-----------------------------------------

void LapManager::writeDataToFile (Runner &runner) {
  ros::Time time = ros::Time::now();
    
  // How long we have been running
  double secs = time.toSec() - start_time_.toSec();
  // Get the hours
  int hours = floor(secs/3600);
  // Get the minutes
  int minutes = floor(secs/60);
  minutes = minutes % 60;
  // Get the seconds
  secs = secs - minutes*60 - hours*3600;

  // Output data to file
  data_file_ << "\n";
  data_file_ << std::fixed << std::setprecision(2);
  data_file_ << std::setfill('0') << std::setw(2) << hours << ":" ;
  data_file_ << std::setfill('0') << std::setw(2) << minutes << ":";
  data_file_ << std::setfill('0') << std::setw(5) << secs << "\t";
  data_file_ << std::fixed << std::setprecision(3);
  data_file_ << std::setfill('0') << std::setw(6) << runner.pos_x_ << "\t\t";
  data_file_ << std::setfill('0') << std::setw(6) << runner.pos_y_ << "\t\t";
  data_file_ << runner.vel_x_ << "\t\t";
  data_file_ << runner.vel_theta_ << "\t\t";
  data_file_ << std::fixed << std::setprecision(2);
  data_file_ << std::setfill('0') << std::setw(5) << runner.batt_voltage_ << "\t\t";
}// end writeDataToFile

//-----------------------------------------

void LapManager:: dockWhenClose(Runner &runner) {
  // Check if we're close to dock
  double delta_x = std::abs(runner.pos_x_ - dock_x_);
  double delta_y = std::abs(runner.pos_y_ - dock_y_);
        
  // Only start docking once we are close enough
  if (delta_x < dock_proximity_ && delta_y < dock_proximity_) {
    ROS_INFO("Going to try and dock");
    runner.setActionGoal(dock_x_, dock_y_, dock_yaw_);
    runner.sendActionGoal();
    runner.dock();
  }// end if
}// end dockWhenClose



