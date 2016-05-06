// Class definition for "Runner"

#ifndef RUNNER_H
#define RUNNER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingGoal.h>
#include <kobuki_msgs/SensorState.h>

//---------------------------------------------------------------------------

class Runner {

  public:
    
    // Kobuki battery
    double batt_voltage_;
    bool charging_;    
    bool full_charge_;
    
    // Goal details
    geometry_msgs::PoseStamped simple_goal_;
    move_base_msgs::MoveBaseGoal action_goal_;
    actionlib::SimpleClientGoalState nav_state_;
    
    // Pose details
    geometry_msgs::PoseWithCovarianceStamped pose_;
    geometry_msgs::PoseWithCovarianceStamped start_pose_;
    
    // Odom details
    nav_msgs::Odometry odom_;
    
    // Docking details
    actionlib::SimpleClientGoalState docking_state_;

    //-----------------------------------------
    
    // Constructor
    Runner();
    
    // Assign a goal
    void setSimpleGoal(double x_in, double y_in, double yaw_in);
    void setActionGoal(double x_in, double y_in, double yaw_in);
    
    // Send a goal
    void sendSimpleGoal();
    void sendActionGoal();
    
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
    ros::Time time_;
    kobuki_msgs::AutoDockingGoal dock_goal_;
    
    // Create action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
    actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> docking_ac_;
    
    // Create service clients
    ros::ServiceClient costmap_client_;
    std_srvs::Empty costmap_srv_;
    
    // Subscribers
    ros::Subscriber pose_sub_;		// Pose from AMCL
    ros::Subscriber odom_sub_;		// From base
    ros::Subscriber sensors_sub_;	// From kobuki sensors/core
    
    // Publishers
    ros::Publisher goal_pub_;		// For simple goals
    
    //-----------------------------------------
    
    // Callback for pose subscriber
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose_cb);
    
    // Callback for odom subscriber
    void odomPoseCallback(const nav_msgs::Odometry &odom_cb);
    
    // Callback for battery subscriber
    void sensorsCallback(const kobuki_msgs::SensorState &sensors_cb);
    
};// end Runner class definition

#endif

