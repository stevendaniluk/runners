// Class implementation for "Runner"

#include <runner.h>

//---------------------------------------------------------------------------

Runner::Runner() : nav_state_(actionlib::SimpleClientGoalState::LOST, "nav_state"), 
                   docking_state_(actionlib::SimpleClientGoalState::LOST, "docking_state"),
                   move_base_ac_("move_base", true), 
                   docking_ac_("dock_drive_action", true) {

  // Subscribe to pose
  pose_sub_ = nh_.subscribe("/amcl_pose", 100, &Runner::amclPoseCallback, this);
  
  // Subscribe to odom
  odom_sub_ = nh_.subscribe("/odom", 100, &Runner::odomPoseCallback, this);
  
  // Subscribe to kobuki sensors
  sensors_sub_ = nh_.subscribe("/mobile_base/sensors/core", 1, &Runner::sensorsCallback, this);
  
  // Subscribe to mobile base velocity
  vel_sub_ = nh_.subscribe("/mobile_base/commands/velocity", 100, &Runner::velocityCallback, this);
  
  // Form publisher to simple goals
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
  
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>
                            ("/navigation_velocity_smoother/raw_cmd_vel", 1000);
  
  // Initialize costmap client
  costmap_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  
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
void Runner::setActionGoal(double x_in, double y_in, double yaw_in) {
  
  action_goal_.target_pose.pose.position.x = x_in;
  action_goal_.target_pose.pose.position.y = y_in;
  
  tf::Quaternion yaw_quat = tf::createQuaternionFromYaw(yaw_in);
  action_goal_.target_pose.pose.orientation.x = yaw_quat.x();
  action_goal_.target_pose.pose.orientation.y = yaw_quat.y();
  action_goal_.target_pose.pose.orientation.z = yaw_quat.z();
  action_goal_.target_pose.pose.orientation.w = yaw_quat.w();

  action_goal_.target_pose.header.frame_id = "map";
  
}// end setCurrentGoal

//-----------------------------------------

// Assign a goal
void Runner::setSimpleGoal(double x_in, double y_in, double yaw_in) {
  
  simple_goal_.pose.position.x = x_in;
  simple_goal_.pose.position.y = y_in;
  
  tf::Quaternion yaw_quat = tf::createQuaternionFromYaw(yaw_in);
  simple_goal_.pose.orientation.x = yaw_quat.x();
  simple_goal_.pose.orientation.y = yaw_quat.y();
  simple_goal_.pose.orientation.z = yaw_quat.z();
  simple_goal_.pose.orientation.w = yaw_quat.w();
  
  simple_goal_.header.frame_id = "map";
  
}// end setCurrentGoal

//-----------------------------------------

// Send the current goal (without monitoring progress)
void Runner::sendSimpleGoal() {
  
  // Timestamp
  simple_goal_.header.stamp = ros::Time::now();
  
  ros::Rate poll_rate(50);
  // Wait until our subscriber is up
  while(goal_pub_.getNumSubscribers() == 0) {
    poll_rate.sleep();
  }// end while
  
  // Send the goal
  goal_pub_.publish(simple_goal_);
  
}// end sendGoal

//-----------------------------------------

// Send an action goal and wait
void Runner::sendActionGoal() {
  
  // Timestamp
  action_goal_.target_pose.header.stamp = ros::Time::now();
  
  // Send the goal
  ROS_INFO("Sending goal.");
  move_base_ac_.sendGoal(action_goal_);
  
  // Monitor status
  while (!move_base_ac_.waitForResult(ros::Duration(3))) {
    nav_state_ = move_base_ac_.getState();
    ROS_INFO("Navigation status: %s",nav_state_.toString().c_str());
  }// end while
  
  updateNavState();
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
    
    if (ros::Time::now() > (time_ + ros::Duration(30))) {
      ROS_INFO("Docking took more than 30 seconds, canceling.");
      docking_ac_.cancelGoal();
      break;
    }// end if
  }// end while
  docking_state_ = docking_ac_.getState();
  
}// end dock

//-----------------------------------------

// Back up from dock and rotate away
void Runner::undock() {
  
  // Form docking parameters
  double back_up_dist = 0.3;	// [m]
  double back_up_speed = 0.25;	// [m/s]
  ros::Duration back_up_time (back_up_dist / back_up_speed);
  
  double rotation_angle = 3.1416;	// [rad]
  double rotation_speed = 1.0;		// [rad/s]
  ros::Duration rotation_time (rotation_angle / rotation_speed);
  
  // Form messages
  geometry_msgs::Twist back_up_msg;
  back_up_msg.linear.x = -back_up_speed;
  
  geometry_msgs::Twist rotation_msg;
  rotation_msg.angular.z = rotation_speed;
  
  ros::Time start_time = ros::Time::now();
  ros::Rate loop_rate(20);
  
  ROS_INFO("Backing up from dock.");
  
  while (ros::Time::now() < (start_time + back_up_time)) {
    cmd_vel_pub_.publish(back_up_msg);
    loop_rate.sleep();
  }// end while
  
  start_time = ros::Time::now();
  
  while (ros::Time::now() < (start_time + rotation_time)) {
    cmd_vel_pub_.publish(rotation_msg);
    loop_rate.sleep();
  }// end while
  
}// end undock

//-----------------------------------------

// Clear costmap
void Runner::clearCostmap() {
  
  bool success = costmap_client_.call(costmap_srv_);
  if (success) {
    ROS_INFO("Costmap cleared.");
  }else {
    ROS_ERROR("Clearing costmap failed.");
  }// end if
  
}// end clearCostmaps

//-----------------------------------------

// Update the callbacks
void Runner::update() {
  ros::spinOnce();
}// end update

//-----------------------------------------

// Callback for pose subscriber
void Runner::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_cb){
  pos_x_ = pose_cb.pose.pose.position.x;
  pos_y_ = pose_cb.pose.pose.position.y;
}// end pose callback

//-----------------------------------------

// Callback for odom subscriber
void Runner::odomPoseCallback(const nav_msgs::Odometry &odom_cb){
  cov_xx_ = odom_cb.pose.covariance[0];
  cov_yy_ = odom_cb.pose.covariance[7];
}// end pose callback

//-----------------------------------------

void Runner::velocityCallback(const geometry_msgs::Twist &vel_cb) {
  vel_x_ = vel_cb.linear.x;
  vel_theta_ = vel_cb.angular.z;
}// end callback

//-----------------------------------------

// Callback for sensors
void Runner::sensorsCallback(const kobuki_msgs::SensorState &sensors_cb) {
  
  // Get the voltage
  batt_voltage_ = double(sensors_cb.battery);
  batt_voltage_ = batt_voltage_/10;
  
  if (batt_voltage_ < 14.0) {
    batt_low_ = true;
  }else {
    batt_low_ = false;
  }// end if
  
  // Set if we are charging
  if (sensors_cb.charger != 0) {
    charging_ = true;
  }else {
    charging_ = false;
  }// end if
  
  // Set if we are in the dock
  if (sensors_cb.charger == 6) {
    docked_ = true;
  }else {
    docked_ = false;
  }// end if
  
  // Set if we are fully charged
  if (sensors_cb.charger == 2 || sensors_cb.charger == 18) {
    full_charge_ = true;
  }else {
    full_charge_ = false;
  }// end if
  
}// end sensors callback



