// Test for using Runner class

#include <ros/ros.h>
#include <runner.h>
#include <cstdlib>

//---------------------------------------------------------------------------

int main(int argc, char** argv){
  
  ros::init(argc, argv, "class_test");
  
  // Check input arguments
  if (argc != 4 && argc!=5) {
    ROS_INFO("Usage: Input arguments [x, y, theta]");
    return 1;
  }// end if
  
  ROS_INFO("I'm ALIIIVVEEEEE!!!");
  
  // Create Runner object
  Runner test;
  
  test.showPose();
    
  // Output id
  ROS_INFO("Runner id is: %d", test.id);
  // Change id
  test.setId(3);
  // Output new id
  ROS_INFO("Runner id is: %d", test.id);
  
  // Assign input arguments to goal object
  test.setCurrentGoal(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  
  // Assign goal frame, if frame argument is given
  if (argc == 5) {
    test.current_goal.target_pose.header.frame_id = argv[4];
  }// end if
  
  // Send a goal
  test.sendGoal(test.current_goal);
  
  test.showPose();
  
  return 0;
}


