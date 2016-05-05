// Class implementation for "Runner"

#include <runner.h>

//---------------------------------------------------------------------------

Runner::Runner() : nav_state_(actionlib::SimpleClientGoalState::LOST, "nav_state"), 
                   docking_state_(actionlib::SimpleClientGoalState::LOST, "docking_state"),
                   move_base_ac_("move_base", true), 
                   docking_ac_("dock_drive_action", true) {

  // Subscribe to pose
  pose_sub_ = nh.subscribe("amcl_pose", 100, &Runner::amclPoseCallback, this);
  
  // Subscribe to odom
  odom_sub_ = nh.subscribe("odom", 100, &Runner::odomPoseCallback, this);
  
  // Subscribe to kobuki sensors
  sensors_sub_ = nh.subscribe("mobile_base/sensors/core", 1, &Runner::sensorsCallback, this);
  
  // Initialize costmap client
  costmap_client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
  // Wait for action servers (do individually in case docking is not used)
  ROS_INFO("Waiting for action servers to come up");
  int loop_counter=0;
  
  while(!move_base_ac_.waitForServer(ros::Duration(1.0))) {
    if (loop_counter >= 3) {
      ROS_INFO("The move_base action server did not successfully come up");
      break;
    }// end if
    loop_counter++;
  }// end while
  
  loop_counter=0;
  while(!docking_ac_.waitForServer(ros::Duration(1.0))) {
    if (loop_counter >= 3) {
      ROS_INFO("The docking action server did not successfully come up");
      break;
    }// end if
    loop_counter++;
  }// end while
  
  // Update to get callbacks
  update();
  
}// end constructor

//-----------------------------------------

// Assign a goal
void Runner::setCurrentGoal(float x_in, float y_in, float theta_in, float w_in) {
  current_goal_.target_pose.pose.position.x=x_in;
  current_goal_.target_pose.pose.position.y=y_in;
  current_goal_.target_pose.pose.orientation.z=theta_in;
  current_goal_.target_pose.pose.orientation.w = w_in;
  current_goal_.target_pose.header.frame_id = "map";
}// end setCurrentGoal

//-----------------------------------------

// Send a goal
void Runner::sendGoal(move_base_msgs::MoveBaseGoal &goal) {

  // Timestamp
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Send the goal
  ROS_INFO("Sending goal.");
  move_base_ac_.sendGoal(goal);
  
}// end sendGoal

//-----------------------------------------

// Send a goal and wait
void Runner::sendGoalAndWait(move_base_msgs::MoveBaseGoal &goal) {

  // Timestamp
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Send the goal
  ROS_INFO("Sending goal.");
  move_base_ac_.sendGoal(goal);
  
  // Monitor status
  while (!move_base_ac_.waitForResult(ros::Duration(3))) {
    nav_state_ = move_base_ac_.getState();
    ROS_INFO("Navigation status: %s",nav_state_.toString().c_str());
  }// end while
  
  nav_state_ = move_base_ac_.getState();
  // Check if successful
  if(nav_state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Goal reached successfully.");
  }else {
    ROS_INFO("Navigation to goal failed.");
  }// end if
  
}// end sendGoalAndWait

//-----------------------------------------

// Update navigation state
void Runner::updateNavState() {
  nav_state_ = move_base_ac_.getState();
}// end sendGoal

//-----------------------------------------

// Set start pose
void Runner::setStartPose() {
  ros::spinOnce();
  start_pose_ = pose_;
}// end showPose

//-----------------------------------------

// Start auto docking
void Runner::dock() {
  
  // Send the goal
  ROS_INFO("Begin docking.");
  docking_ac_.sendGoal(dock_goal_);
  
  time_ = ros::Time::now();
  
  // Monitor status
  while (!docking_ac_.waitForResult(ros::Duration(3))) {
    docking_state_ = docking_ac_.getState();
    ROS_INFO("Docking status: %s",docking_state_.toString().c_str());
    
    if (ros::Time::now() > (time_+ros::Duration(30))) {
      ROS_INFO("Docking took more than 30 seconds, canceling.");
      docking_ac_.cancelGoal();
      break;
    }// end if
  }// end while
  docking_state_ = docking_ac_.getState();
}// end dock

//-----------------------------------------

// Clear costmap
void Runner::clearCostmap() {
  
  bool success = costmap_client_.call(costmap_srv_);
  if (success) {
    ROS_INFO("Costmap cleared.");
  }else {
    ROS_INFO("Clearing costmap failed.");
  }// end if
}// end clearCostmaps

//-----------------------------------------

// Update the callbacks
void Runner::update() {
  ros::spinOnce();
}// end update

//-----------------------------------------

// Callback for pose subscriber
void Runner::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped & pose_cb){
  pose_ = pose_cb;
}// end pose callback

//-----------------------------------------

// Callback for odom subscriber
void Runner::odomPoseCallback(const nav_msgs::Odometry & odom_cb){
  odom_ = odom_cb;
}// end odom callback

//-----------------------------------------

// Callback for sensors
void Runner::sensorsCallback(const kobuki_msgs::SensorState & sensors_cb) {
  // Get the voltage
  batt_voltage_ = double(sensors_cb.battery);
  batt_voltage_ = batt_voltage_/10;
  
  // Set if we are charging
  if (sensors_cb.charger != 0) {
    charging_ = true;
  }else {
    charging_ = false;
  }// end if
  
  // Set if we are fully charged
  if (sensors_cb.charger == 2 || sensors_cb.charger == 18) {
    full_charge_ = true;
  }else {
    full_charge_ = false;
  }// end if
}// end sensors callback

