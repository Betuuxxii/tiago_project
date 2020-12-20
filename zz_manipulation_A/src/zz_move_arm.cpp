// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


// Std C++ headers
#include <string>
#include <vector>
#include <map>

#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

#include <std_srvs/Empty.h>


geometry_msgs::PoseStamped object_pose;
geometry_msgs::PoseStamped object_pose_callback;
geometry_msgs::PoseStamped goal_pose;

bool move_success= true;

int cont_callback;

std_msgs::Bool enabled;

ros::Publisher pub_cmd_vel;

void clearOctomap(){
    ROS_INFO("Clear octomap");
    std_srvs::Empty emptymsg;
    ros::service::call("/clear_octomap",emptymsg);
}

void runPlayMotion(std::string motion_name){

  ROS_INFO("Starting run_motion application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    //return EXIT_FAILURE;
  }

  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);

  ROS_INFO("Waiting for Action Server ...");
  client.waitForServer();

  play_motion_msgs::PlayMotionGoal goal;

  goal.motion_name = motion_name;
  goal.skip_planning = false;
  goal.priority = 0;

  ROS_INFO_STREAM("Sending goal with motion: " << motion_name);
  client.sendGoal(goal);

  ROS_INFO("Waiting for result ...");
  bool actionOk = client.waitForResult(ros::Duration(30.0));

  actionlib::SimpleClientGoalState state = client.getState();

  if ( actionOk )
  {
      ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
  }
  else
  {
      ROS_ERROR_STREAM("Action failed with state: " << state.toString());
  }

}

void takeObject(geometry_msgs::PoseStamped object_pose){
  move_success= false; 

  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = object_pose.pose.position.x - 0.18 ;
  goal_pose.pose.position.y = object_pose.pose.position.y + 0.02;//0.03
  goal_pose.pose.position.z = object_pose.pose.position.z + 0.03;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5,0,0);//-1.2,1.5,-1.2);//-3.14 / 2, -3.14 / 4, -3.14 / 2);


  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  ROS_INFO_STREAM("declalrem moveit");
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

    ROS_INFO_STREAM("passem moveit");

  //choose your preferred planner
  group_arm_torso.setPlannerId("RRTkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(2.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if ( !success ){
    ROS_ERROR("No plan found");
        move_success = true;

  }
  else{
    try{
      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();


      moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
     if (!bool(e)){
          ROS_ERROR("Error executing plan");

        }
        else{
        move_success = true;
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        }

    }
    catch (...){
    ROS_ERROR("Error executing plan");

    //intent n2
    ros::Duration(1).sleep();
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();

    move_success = true;
    }


  }

  runPlayMotion("close");


  spinner.stop();




  }

void goToObject(geometry_msgs::PoseStamped object_pose, float x, float y, float z){

  move_success= false; 

  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = object_pose.pose.position.x + x ;
  goal_pose.pose.position.y = object_pose.pose.position.y + y;
  goal_pose.pose.position.z = object_pose.pose.position.z + z;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5,0,0);//-1.2,1.5,-1.2);//-3.14 / 2, -3.14 / 4, -3.14 / 2);


  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  ROS_INFO_STREAM("declalrem moveit");
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

    ROS_INFO_STREAM("passem moveit");

  //choose your preferred planner
  group_arm_torso.setPlannerId("RRTkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  //group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(3.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if ( !success ){
    ROS_ERROR("No plan found");
        move_success = true;

  }
  else{
    try{
      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();


      moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
     if (!bool(e)){
          ROS_ERROR("Error executing plan");
        }
        else{
        //move_success = true;
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        }

    }
    catch (...){
    ROS_ERROR("Error executing plan");

    //intent n2
    ros::Duration(1).sleep();
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();

    move_success = true;
    }


  }

  spinner.stop();


}



void leaveObject(){

  geometry_msgs::Twist cmd_vel;

  for(int i=0;i<5;i++){
    cmd_vel.linear.x = -1;
    pub_cmd_vel.publish(cmd_vel);
    ros::Duration(0.7).sleep();

  }

  clearOctomap();

  move_success= false; 

  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = 0.45;//0.498;
  goal_pose.pose.position.y = -0.15;//-0.069;
  goal_pose.pose.position.z = 0.5;//0.481;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,1.5);


  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  ROS_INFO_STREAM("declalrem moveit");
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

  ROS_INFO_STREAM("passem moveit");

  //choose your preferred planner
  group_arm_torso.setPlannerId("RRTkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  //group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if ( !success ){
    ROS_ERROR("No plan found");

    bool success = bool(group_arm_torso.plan(my_plan));

      if ( !success ){
    ROS_ERROR("No plan found 2");}
    move_success = true;

  }
  else{
    try{
      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();


      moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
     if (!bool(e)){
          ROS_ERROR("Error executing plan");
        }
        else{
        //move_success = true;
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        }

    }
    catch (...){
    ROS_ERROR("Error executing plan");

    move_success = true;
    }


  }

  spinner.stop();


}


void callback_enable(std_msgs::Bool bool_msgs){

  enabled.data = bool_msgs.data;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_manipulation");

  ros::NodeHandle nh;

  tf::TransformListener listener;
  
  //ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/zz_cylinder_detector/cylinder_pose", 1, callbackCylinderPose);

  ros::Subscriber sub_enable = nh.subscribe<std_msgs::Bool>("/enable_manipulation", 1, callback_enable);
  ros::Publisher pub_enable_manipulation = nh.advertise<std_msgs::Bool>("/enable_manipulation", 1);
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);


  ros::Rate loop_rate(10);
 
  std_msgs::Bool enable;
  
  enable.data = false;

  //planningScene(1.0);
  ros::Duration(1.0).sleep();
  
  /*runPlayMotion("close");
  runPlayMotion("pick_final_pose");
  leaveObject();
*/
  // Hands up

  while(ros::ok() )
  {

     
      if (enabled.data == true ){//and move_success){//and object_pose.pose.position.x > 0){
      clearOctomap();
      runPlayMotion("head_tour");    
      runPlayMotion("pregrasp");      
      runPlayMotion("open");      
      

      //if(move_success){
          tf::StampedTransform transform2;  
        
          try{
            listener.lookupTransform("/base_footprint", "/cluster_link",  ros::Time(0), transform2);
          }
          catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(2.0).sleep();
            /*  try{
                listener.lookupTransform("/base_footprint", "/cluster_link",  ros::Time(0), transform2);
              }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());

              }*/
          }

          if (transform2.getOrigin().z() > 0.65){
            object_pose.pose.position.x = transform2.getOrigin().x();
            object_pose.pose.position.y = transform2.getOrigin().y();
            object_pose.pose.position.z = transform2.getOrigin().z();
            object_pose.pose.orientation.x = transform2.getRotation().x();
            object_pose.pose.orientation.y = transform2.getRotation().y();
            object_pose.pose.orientation.z = transform2.getRotation().z();
            object_pose.pose.orientation.w = transform2.getRotation().w();

            std::cout << "Object position: "<< object_pose.pose << "\n" << std::endl;

          }
      // }
      

      goToObject(object_pose, -0.35,0.03,0.03);  
      //ros::Duration(1.0).sleep();
      takeObject(object_pose);
      //ros::Duration(1.0).sleep();
      goToObject(object_pose, -0.18,0.03,0.18);  

      runPlayMotion("pick_final_pose");

      ROS_INFO("leaveObject");
      leaveObject();
      //ros::Duration(60).sleep();
      runPlayMotion("gir"); 
      runPlayMotion("open"); 
      runPlayMotion("initial"); 

      enabled.data = false;
      pub_enable_manipulation.publish(enabled);
      ros::Duration(1.0).sleep();
 

      }
      

      ros::spinOnce();
    
  }

  ros::spin();


  return EXIT_SUCCESS;
}
