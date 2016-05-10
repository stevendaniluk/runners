// Runners Planner for following a loop

#include <runners_planner/runners_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(runners_planner::RunnersPlanner, nav_core::BaseGlobalPlanner)

namespace runners_planner {
  
  // Constructor
  RunnersPlanner::RunnersPlanner() : costmap_ros_(NULL), initialized_(false){}
  
  //----------------------------------------
  
  // Constructor
  //
  // INPUTS
  // name = The name of the planner
  // costmap_ros = A pointer to the ROS wrapper of the costmap to use for planning
  RunnersPlanner::RunnersPlanner (std::string name, costmap_2d::Costmap2DROS* costmap_ros)
                                 : costmap_ros_(NULL), initialized_(false){
    
    initialize(name, costmap_ros);
    
  }// end constructor
  
  //----------------------------------------
  
  // Initialize
  //
  // INPUTS
  // name = The name of the planner
  // costmap_ros = A pointer to the ROS wrapper of the costmap to use for planning
  void RunnersPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      world_model_ = new base_local_planner::CostmapModel(*costmap_); 
      
      // Create the executive planner nodehandle in global namespace for params
      ros::NodeHandle exec_planner_nh("runners/ExecutivePlanner");
      
      // Get parameters
      exec_planner_nh.param("dock_x", dock_x_, 0.00);
      exec_planner_nh.param("dock_y", dock_y_, 0.00);
      exec_planner_nh.param("dock_yaw", dock_yaw_, 0.00);
      exec_planner_nh.param("pt_spacing", pt_spacing_, 0.01);
      exec_planner_nh.param("planning_distance", planning_distance_, 2.0);
      if (!exec_planner_nh.getParam("path_filename", path_filename_)){
        ROS_ERROR("Failed to get param 'path_filename'. A text file with coordinates is necessary for this planner to run.");
      }// end if
      if (!exec_planner_nh.getParam("path_package", path_package_)){
        ROS_ERROR("Failed to get param 'path_package'. This is necessary in order to find the path file.");
      }// end if
      
      // Set inputted file name and path
      std::string full_file = ros::package::getPath(path_package_) + "/" + path_filename_;
    
      // Load file
      std::ifstream is(full_file.c_str());
      std::istream_iterator<double> start(is), end;
      std::vector<double> data(start, end);
      
      // Resize vectors appropriately
      int columns = 2;
      num_pts_ = data.size()/columns;
      path_x_.resize(num_pts_);
      path_y_.resize(num_pts_);
  
      // Sort data columns into vectors
      for (int i =0 ; i < num_pts_ ; i++){
        path_x_[i] = data[columns*i];
        path_y_[i] = data.at(columns*i+1);
       }// end for
    
      initialized_ = true;
      
      ROS_INFO("Created global_planner runners_planner");
      ROS_INFO("Loaded path file");
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }// end initialize
  
  //----------------------------------------
  
  // footprintCost
  //
  // Checks the legality of the robot footprint at a position and orientation 
  // using the world model. Because we need to take the footprint of the robot 
  // into account when we calculate cost to obstacles
  //
  // INPUTS
  // x, y, and theta are the position and orientation to evaluate the cost at
  
  double RunnersPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    } //end if

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3) {
      return -1.0;
    }// end if

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }// end foorprintCost
  
  //----------------------------------------
  
  // makePlan
  //
  // INPUTS
  // start = Start point for plan (not used)
  // goal = End point for plan (not used)
  // plan = Reference to plan object (vector of poses) to be filled out

  bool RunnersPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    
    // Check the initialization
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }// end if
    
    // Make sure the plan vector is empty
    plan.clear();
    
    // First, handle whether we are going to the dock or not
    if (goal.pose.position.x == dock_x_ && goal.pose.position.y == dock_y_) {
      // Go to the dock
      ROS_DEBUG("Goal set to docking location.");
      
      plan.push_back(start);
      plan.push_back(goal);
      return (true);
    }// end if
    
    // Check if goal and costmap frame ID's match
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }// end if
    
    ROS_DEBUG("Starting path generation");
    ROS_DEBUG("Finding closest point on path");
    
    // Create vectors to hold the difference to each point
    std::vector<double> x_diff (num_pts_);
    std::vector<double> y_diff (num_pts_);
  
    // Initialize the current minimum distance to the first path point
    double dist_x = (start.pose.position.x - path_x_[0]);
    double dist_y = (start.pose.position.x - path_y_[0]);
    double dist_min = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
    
    // For storing what the minimum distance is, and where it is
    double dist;
    int index = 0;
    
    // Check each point
    for(int i = 0 ; i < num_pts_; i++){
      dist_x = (start.pose.position.x - path_x_[i]);
      dist_y = (start.pose.position.y - path_y_[i]);
      dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
	  
      if (dist < dist_min){
        dist_min = dist;
        index = i;
      }// end if
    }// end for
    
    ROS_DEBUG("Start point #%d, X=%.3f, Y=%.3f", index + 1, path_x_[index], path_y_[index]);

    // Creat temp goal to feed into plan (equal to goal because we need the tf info)
    geometry_msgs::PoseStamped temp_goal = goal;
    
    // How many points we want to look ahead
    int pts_ahead;
    if (planning_distance_ < 0) {
      // Loop to within 0.5m of start point, so it doesn't think we've reached our goal
      pts_ahead = num_pts_ - 0.5 / pt_spacing_;
    }else {
      pts_ahead = planning_distance_ / pt_spacing_;
    }// end if
    
    // Assign all the goal points
    for (int i = 0 ; i < pts_ahead;  i++) {
      
      // Increment, and loop back to zero if we've surpassed num_pts
      index++;
      index = index % num_pts_;
      
      // Fill out temp goal
      temp_goal.pose.position.x = path_x_[index];
      temp_goal.pose.position.y = path_y_[index];
      
      // Must assign a proper orientation for the last point
      if (i == (pts_ahead - 1)){
        double x_component = path_x_[(index + 1) % num_pts_] - path_x_[index];
        double y_component = path_y_[(index + 1) % num_pts_] - path_y_[index];
        double yaw = atan2(y_component, x_component);
        
        tf::Quaternion temp_goal_quat = tf::createQuaternionFromYaw(yaw);
        temp_goal.pose.orientation.x = temp_goal_quat.x();
        temp_goal.pose.orientation.y = temp_goal_quat.y();
        temp_goal.pose.orientation.z = temp_goal_quat.z();
        temp_goal.pose.orientation.w = temp_goal_quat.w();   
      }// end if
      
      plan.push_back(temp_goal);
    }// end for
    
    ROS_DEBUG("End point #%d, X=%.3f, Y=%.3f", index + 1, path_x_[index], path_y_[index]);

    return (true);
    
  }// end makePlan

};
