// Class implementation for "Runner"

#include <runner.h>

//---------------------------------------------------------------------------

Runner::Runner() : nav_state(actionlib::SimpleClientGoalState::LOST, "test"), 
                   docking_state(actionlib::SimpleClientGoalState::LOST, "test"),
                   move_base_ac("move_base", true), 
                   docking_ac("dock_drive_action", true) {

  // Subscribe to pose
  pose_sub = nh.subscribe("amcl_pose", 100, &Runner::amcl_pose_callback, this);
  
  // Subscribe to odom
  odom_sub = nh.subscribe("odom", 100, &Runner::odom_pose_callback, this);
  
  // Initialize costmap client
  costmap_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
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
  
  // Update
  update();
  
}// end constructor

//-----------------------------------------

// Assign a goal
void Runner::setCurrentGoal(float x_in, float y_in, float theta_in, float w_in) {
  current_goal.target_pose.pose.position.x=x_in;
  current_goal.target_pose.pose.position.y=y_in;
  current_goal.target_pose.pose.orientation.z=theta_in;
  current_goal.target_pose.pose.orientation.w = w_in;
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

// Send a temporary goal
void Runner::sendTempGoal(move_base_msgs::MoveBaseGoal &goal) {

  // Timestamp
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Send the goal
  move_base_ac.sendGoal(goal);
  
  // Update the state
  nav_state = move_base_ac.getState();
  
}// end sendGoal

//-----------------------------------------

// Update nav_state
void Runner::updateNavState() {
  nav_state = move_base_ac.getState();
}// end sendGoal

//-----------------------------------------

// Callback for pose subscriber
void Runner::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped & pose_cb){
  pose=pose_cb;
}// end pose callback

//-----------------------------------------

// Callback for odom subscriber
void Runner::odom_pose_callback(const nav_msgs::Odometry & odom_cb){
  odom=odom_cb;
}// end pose callback

//-----------------------------------------

// Update the callbacks
void Runner::update() {
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
  docking_state = docking_ac.getState();
}// end dock

//-----------------------------------------

// Clear costmap
void Runner::clearCostmap() {
  
  bool success = costmap_client.call(costmap_srv);
  if (success) {
    ROS_INFO("Costmap cleared.");
  }else {
    ROS_INFO("Clearing costmap failed.");
  }// end if
}

