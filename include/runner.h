// Class definition for "Runner"

#ifndef RUNNER_H
#define RUNNER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingGoal.h>
#include "kobuki_msgs/SensorState.h"

//---------------------------------------------------------------------------

class Runner {

  public:
    
    // Kobuki battery
    double batt_voltage;
    bool charging;    
    bool full_charge;
    
    // Goal details
    move_base_msgs::MoveBaseGoal current_goal;
    actionlib::SimpleClientGoalState nav_state;
    
    // Pose details
    geometry_msgs::PoseWithCovarianceStamped pose;
    geometry_msgs::PoseWithCovarianceStamped start_pose;
    
    // Odom details
    nav_msgs::Odometry odom;
    nav_msgs::Odometry last_known_odom;
    
    // Docking details
    actionlib::SimpleClientGoalState docking_state;
    bool docked;

    //-----------------------------------------
    
    // Constructor
    Runner();
    
    // Assign a goal
    void setCurrentGoal(float x_in, float y_in, float theta_in, float w_in);
    
    // Send a goal
    void sendGoal(move_base_msgs::MoveBaseGoal &goal);
    void sendGoalAndWait(move_base_msgs::MoveBaseGoal &goal);
    
    // Update the callbacks
    void update();
    
    // Set the start pose
    void setStartPose();
    
    // Start docking
    void dock();
    
    // Update nav_state
    void updateNavState();
    
    // Clear costmap
    void clearCostmap();
    
  private:
  
    // Create nodehandle(s)
    ros::NodeHandle nh;
 
    // Member decleration
    ros::Time time;
    kobuki_msgs::AutoDockingGoal dock_goal;
    
    // Create action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac;
    actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> docking_ac;
    
    // Create service clients
    ros::ServiceClient costmap_client;
    std_srvs::Empty costmap_srv;
    
    // Subscribers
    ros::Subscriber pose_sub;		// Pose from AMCL
    ros::Subscriber odom_sub;		// From base
    ros::Subscriber sensors_sub;	// From kobuki sensors/core
    
    //-----------------------------------------
    
    // Callback for pose subscriber
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_cb);
    
    // Callback for odom subscriber
    void odomPoseCallback(const nav_msgs::Odometry &odom_cb);
    
    // Callback for battery subscriber
    void sensorsCallback(const kobuki_msgs::SensorState &sensors_cb);
    
};// end Runner class definition

#endif

