#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Bool.h"
#include "nav_msgs/GetPlan.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goal;
ros::ServiceClient check_path ;
nav_msgs::GetPlan srv;

move_base_msgs::MoveBaseActionFeedback feedback_;
actionlib_msgs::GoalID goalid;

main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "tiago_navigation");
    ros::NodeHandle nh;

    check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    while(!ac.waitForServer(ros::Duration(5.0)));

         
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =   4;
    goal.target_pose.pose.position.y =   1.2;
    goal.target_pose.pose.orientation.w = 1.0;

    //Servei goal comprovem sortida
    srv.request.start = feedback_.feedback.base_position;
    srv.request.goal = goal.target_pose;
    srv.request.tolerance = 0.4;
    check_path.call(srv);
    std::cout << "Size call service " << srv.response.plan.poses.size() << std::endl;

    if (srv.response.plan.poses.size() == 0){
      
      ROS_INFO("No plan found.");
    }

    ac.sendGoalAndWait(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Goal reached");
    }

    
    while(ros::ok()){
      ros::spinOnce();

    }

    return 0;


}
