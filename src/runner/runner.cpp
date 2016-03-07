// Class implementation for "Runner"

#include <runner.h>

//---------------------------------------------------------------------------

Runner::Runner() : move_base_ac("move_base", true), docking_ac("dock_drive_action", true) {
  
  // Wait for action server
  while(!move_base_ac.waitForServer(ros::Duration(5.0)) || 
        !docking_ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for action servers to come up");
  }// end while

  
  // Subscribe to pose
  pose_sub = nh.subscribe("amcl_pose", 100, &Runner::amcl_pose_callback, this);
  
}// end constructor

//-----------------------------------------

// Assign a goal
void Runner::setCurrentGoal(float x_in, float y_in, float theta_in) {
  current_goal.target_pose.pose.position.x=x_in;
  current_goal.target_pose.pose.position.y=y_in;
  current_goal.target_pose.pose.orientation.z=theta_in*(3.14159265/180);
  current_goal.target_pose.pose.orientation.w = 1.0;
  current_goal.target_pose.header.frame_id = "base_link";
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
    
    // Check if successful
    if(nav_state == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Goal reached successfully.");
    }else {
      ROS_INFO("Navigation to goal failed.");
    }// end if
    
  }// end while
}// end sendGoal

//-----------------------------------------

// Callback for pose subscriber
void Runner::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped & pose_cb){
  pose.pose.pose.position.x = pose_cb.pose.pose.position.x;
  pose.pose.pose.position.y = pose_cb.pose.pose.position.y;
  pose.pose.pose.orientation.z = pose_cb.pose.pose.orientation.z*(180/3.14159265);

}// end pose callback

//-----------------------------------------

// Spinner
void Runner::spin() {
  ros::spinOnce();
}// end spin

// Show pose
void Runner::showPose() {
  ros::spinOnce();
  ROS_INFO("Current pose: x=%f, y=%f, theta=%f\n", pose.pose.pose.position.x,
          pose.pose.pose.position.y, pose.pose.pose.orientation.z);

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
    dock_state = docking_ac.getState();
    ROS_INFO("Docking status: %s",dock_state.toString().c_str());
    
    if (ros::Time::now() > (time+ros::Duration(30))) {
      ROS_INFO("Docking took more than 30 seconds, canceling.");
      docking_ac.cancelGoal();
      break;
    }// end if
  }// end while
}// end dock







