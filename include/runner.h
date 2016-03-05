// Class definition for "Runner"

#ifndef RUNNER_H
#define RUNNER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//---------------------------------------------------------------------------

class Runner {

  public:
    
    // Member decleration
    int id;	// Instance id
    move_base_msgs::MoveBaseGoal current_goal;
    
    geometry_msgs::PoseWithCovarianceStamped pose;
    
    //-----------------------------------------
    
    // Constructor
    Runner();
    
    // Assign id to instance
    void setId(int id_in);
    
    // Assign a goal
    void setCurrentGoal(float x_in, float y_in, float theta_in);
    
    // Send a goal
    void sendGoal(move_base_msgs::MoveBaseGoal &goal);
    
    // Callback for pose subscriber
    void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose);
    
    // Spinner
    void spin();
    
    // Show pose
    void showPose();
    
  private:
  
    // Create nodehandle(s)
    ros::NodeHandle nh;
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    
    ros::Subscriber pose_sub;
    
};// end Runner class definition

#endif

