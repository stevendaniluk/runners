// Class implementation for "Runner"

#include <runner.h>

//---------------------------------------------------------------------------

Runner::Runner() : ac("move_base", true) {
  
  // Wait for action server
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }// end while
  
  // Set instance ID
  setId(1);
  
  // Subscribe to pose
  pose_sub = nh.subscribe("amcl_pose", 100, &Runner::amcl_pose_callback, this);
  
}// end constructor

//-----------------------------------------

// setId member function
void Runner::setId(int id_in) {
 id=id_in;
}// end setId

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
  
  // Send the goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goal);
  ac.waitForResult();
  
  // Check if successful
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Goal reached successfully.");
      //return true;
    }else {
      ROS_INFO("Navigation to goal failed.");
      //return false;
    }// end if

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




