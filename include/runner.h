// Class definition for "Runner"

#ifndef RUNNER_H
#define RUNNER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/AutoDockingGoal.h>

//---------------------------------------------------------------------------

class Runner {

  public:
    
    // Member decleration
    move_base_msgs::MoveBaseGoal current_goal;

    //-----------------------------------------
    
    // Constructor
    Runner();
    
    // Assign a goal
    void setCurrentGoal(float x_in, float y_in, float theta_in);
    
    // Send a goal
    void sendGoal(move_base_msgs::MoveBaseGoal &goal);

    // Spinner
    void spin();
    
    // Show pose
    void showPose();
    
    // Start docking
    void dock();
    
  private:
    
    // Member decleration
    ros::Time time;
    actionlib::SimpleClientGoalState dock_state = actionlib::SimpleClientGoalState::LOST;
    actionlib::SimpleClientGoalState nav_state = actionlib::SimpleClientGoalState::LOST;
    geometry_msgs::PoseWithCovarianceStamped pose;
    kobuki_msgs::AutoDockingGoal dock_goal;
    
    // Create nodehandles
    ros::NodeHandle nh;
    
    // Create action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac;
    actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> docking_ac;
    
    // Subscribers
    ros::Subscriber pose_sub;
    
    //-----------------------------------------
    
    // Callback for pose subscriber
    void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose);
    
};// end Runner class definition

#endif

