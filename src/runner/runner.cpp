// Class implementation for "Runner"

#include <runner.h>

//---------------------------------------------------------------------------

Runner::Runner() : nav_state(actionlib::SimpleClientGoalState::LOST, "test"), 
                   docking_state(actionlib::SimpleClientGoalState::LOST, "test"),
                   move_base_ac("move_base", true), 
                   docking_ac("dock_drive_action", true) {

  // Subscribe to pose
  pose_sub = nh.subscribe("amcl_pose", 100, &Runner::amcl_pose_callback, this);
  
  // Wait for action servers (do individually in case docking is not used)
  ROS_INFO("Waiting for action servers to come up");
  int loop_counter=0;
  
  while(!move_base_ac.waitForServer(ros::Duration(1.0))) {
    if (loop_counter >= 3) {
      ROS_INFO("The move_base action server did not successfully come up");
      break;
    }// end if
    loop_counter++;
  }// end while
  
  loop_counter=0;
  while(!docking_ac.waitForServer(ros::Duration(1.0))) {
    if (loop_counter >= 3) {
      ROS_INFO("The docking action server did not successfully come up");
      break;
    }// end if
    loop_counter++;
  }// end while
  
  // Get current pose
  getPose();
  
}// end constructor

//-----------------------------------------

// Assign a goal
void Runner::setCurrentGoal(float x_in, float y_in, float theta_in) {
  current_goal.target_pose.pose.position.x=x_in;
  current_goal.target_pose.pose.position.y=y_in;
  current_goal.target_pose.pose.orientation.z=theta_in*(3.14159265/180);
  current_goal.target_pose.pose.orientation.w = 1.0;
  current_goal.target_pose.header.frame_id = "map";
}// end setCurrentGoal

//-----------------------------------------

// Send a goal
void Runner::sendGoal(move_base_msgs::MoveBaseGoal &goal) {

  // Timestamp
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Send the goal
  ROS_INFO("Sending goal.");
  move_base_ac.sendGoal(goal);
  
  // Monitor status
  while (!move_base_ac.waitForResult(ros::Duration(3))) {
    nav_state = move_base_ac.getState();
    ROS_INFO("Navigation status: %s",nav_state.toString().c_str());
  }// end while
  
  nav_state = move_base_ac.getState();
  // Check if successful
  if(nav_state == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Goal reached successfully.");
  }else {
    ROS_INFO("Navigation to goal failed.");
  }// end if
  
}// end sendGoal

//-----------------------------------------

// Callback for pose subscriber
void Runner::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped & pose_cb){
  pose=pose_cb;
}// end pose callback

//-----------------------------------------

// Get pose
void Runner::getPose() {
  ros::spinOnce();
}// end showPose

//-----------------------------------------

// Set start pose
void Runner::setStartPose() {
  ros::spinOnce();
  start_pose=pose;
}// end showPose

//-----------------------------------------

// Start auto docking
void Runner::dock() {
  
  // Send the goal
  ROS_INFO("Begin docking.");
  docking_ac.sendGoal(dock_goal);
  
  time = ros::Time::now();
  
  // Monitor status
  while (!docking_ac.waitForResult(ros::Duration(3))) {
    docking_state = docking_ac.getState();
    ROS_INFO("Docking status: %s",docking_state.toString().c_str());
    
    if (ros::Time::now() > (time+ros::Duration(30))) {
      ROS_INFO("Docking took more than 30 seconds, canceling.");
      docking_ac.cancelGoal();
      break;
    }// end if
  }// end while
}// end dock

