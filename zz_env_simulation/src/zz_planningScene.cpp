// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>


void planningScene(double z){

  ros::NodeHandle node_handle;

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "table";
  attached_object.object.header.frame_id = "base_footprint";
  attached_object.object.id = "box";

  geometry_msgs::Pose pose;
  pose.position.x = 1.05;
  pose.position.y = 0;
  pose.position.z =0.32;
  pose.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = z;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  attached_object.object.operation = attached_object.object.ADD;

  //ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene");

  ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/zz_cylinder_detector/cylinder_pose", 1, callbackCylinderPose);

  ros::Rate loop_rate(10);

  ros::Duration(1.0).sleep();


  while(ros::ok())
  {      
    planningScene(0.7);
    ros::spinOnce();
    
  }

  ros::spin();


  return EXIT_SUCCESS;
}
