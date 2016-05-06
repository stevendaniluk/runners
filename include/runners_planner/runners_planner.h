// Runners Planner for following a loop

#ifndef RUNNERS_PLANNER_H_
#define RUNNERS_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <fstream> // For loading path text file

namespace runners_planner{
  
  class RunnersPlanner : public nav_core::BaseGlobalPlanner {
    public:
      
      // Constructor for the CarrotPlanner
      RunnersPlanner();
      
      // Constructor for the CarrotPlanner
      RunnersPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      // Initialization function for the CarrotPlanner
      // param name: The name of this planner
      // param costmap_ros: A pointer to the ROS wrapper of the costmap to use for planning
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


      // makePlan
      //
      // INPUTS
      // start = Start point for plan (not used)
      // goal = End point for plan (not used)
      // plan = Reference to plan object (vector of poses) to be filled out
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    
      
    
    //-----------------------------------------
    
    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      //double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_; // The world model that the controller will use

      // footprintCost
      //
      // Checks the legality of the robot footprint at a position and orientation 
      // using the world model. Because we need to take the footprint of the robot 
      // into account when we calculate cost to obstacles
      //
      // INPUTS
      // x, y, and theta are the position and orientation to evaluate the cost at
  
      double footprintCost(double x_i, double y_i, double theta_i);

      bool initialized_;
      
      // Path file variables
      std::vector<double> path_x_, path_y_;
      int num_pts_;
      
      // Docking variables
      double dock_x_;
      double dock_y_;
      double dock_yaw_;
      
      // Parameters
      double pt_spacing_; // How far apart points are spaced in the path text file [m]
      double planning_distance_; // How far ahead to generate the plan [m]
      std::string path_filename_; // Name to text file with path coordinates to load 
      std::string path_package_; // Package name that the path text file is in

  };
};  
#endif
