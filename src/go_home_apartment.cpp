// Will send kobuki to the dock, and begin docking

// Docking position is:
//x=-0.65, y=0.02, a=-0.15

#include <ros/ros.h>
#include <runner.h>
#include <cstdlib>

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "go_home");
  
  // Create Runner object
  Runner Robot;
  
  // Docking position is:
  //x=-0.65, y=0.02, a=-0.15
  
  // Assign input arguments to goal object
  Robot.setCurrentGoal(-0.65, 0.02, 180);
  
  // Send the goal
  Robot.sendGoal(Robot.current_goal);
  
  ros::Duration(1.0).sleep();
  
  // If successful, start docking
  if (Robot.nav_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Beginning docking.");
      Robot.dock();
  }else {
    ROS_INFO("Did not succeed, will not dock.");
  }// end if (goal)

  return 0;
}
