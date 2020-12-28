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

#include <tf/tf.h>

#include <std_srvs/Empty.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goal;
ros::ServiceClient check_path ;
nav_msgs::GetPlan srv;

move_base_msgs::MoveBaseActionFeedback feedback_;
actionlib_msgs::GoalID goalid;

std_msgs::Bool has_command, enable_manipulation;

geometry_msgs::PoseStamped tableA, tableB, tableC, pacient1, pacient2, centre;

int count_callback;

void initialize_var(){

    enable_manipulation.data = false;

    tableA.header.frame_id = "map";
    tableA.header.stamp = ros::Time::now();
    tableA.pose.position.x = 6.2;//6.35;
    tableA.pose.position.y = -0.85;
    tableA.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-1.5);

    tableB.header.frame_id = "map";
    tableB.header.stamp = ros::Time::now();
    tableB.pose.position.x = 6.6;
    tableB.pose.position.y = -0.05;//-0.1;
    tableB.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    tableC.header.frame_id = "map";
    tableC.header.stamp = ros::Time::now();
    tableC.pose.position.x = 6.65;
    tableC.pose.position.y = 1.85;
    tableC.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    pacient1.header.frame_id = "map";
    pacient1.header.stamp = ros::Time::now();
    pacient1.pose.position.x = 4.8;
    pacient1.pose.position.y = -4.4;
    pacient1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    pacient2.header.frame_id = "map";
    pacient2.header.stamp = ros::Time::now();
    pacient2.pose.position.x = 0.88;
    pacient2.pose.position.y = -5.6;
    pacient2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-1.5);

    centre.header.frame_id = "map";
    centre.header.stamp = ros::Time::now();
    centre.pose.position.x = 5.3;
    centre.pose.position.y = 0.5;
    centre.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);




}

void goToPosition(geometry_msgs::PoseStamped pose){


    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    while(!ac.waitForServer(ros::Duration(5.0)));

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =  pose.pose.position.x;
    goal.target_pose.pose.position.y =   pose.pose.position.y;
    goal.target_pose.pose.orientation = pose.pose.orientation;

    //Servei goal comprovem sortida
    srv.request.start = feedback_.feedback.base_position;
    srv.request.goal = goal.target_pose;
    srv.request.tolerance = 0.4;
    check_path.call(srv);
    std::cout << "Size call service " << srv.response.plan.poses.size() << std::endl;

    if (srv.response.plan.poses.size() == 0){
      
      ROS_INFO("No plan found.");
    }

    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(60));

     if (finished_before_timeout)
         {
           actionlib::SimpleClientGoalState state = ac.getState();
           ROS_INFO("Action finished: %s",state.toString().c_str());
           //Preempting the process
          ac.cancelGoal();

         }
     else
       ROS_INFO("Action did not finish before the time out.");

    //ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Goal reached");
    }
    else 
        ROS_WARN("Goal NOT reached....");

    //ros::Duration(10.0);
}

void callback_enable_manipulation(std_msgs::Bool bool_msgs){

  /*  if(count_callback == 1){

        enable_manipulation.data = bool_msgs.data;
        count_callback=0;
    }
    else{
        count_callback ++;
    }*/

    enable_manipulation.data = bool_msgs.data;


}

constexpr long value(const char *definition)
{
    if (definition && *definition)
    {
        return *definition + value(definition + 1);
    }

    return *definition;
}

main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "tiago_navigation");
    ros::NodeHandle nh;

    ros::Publisher pub_enable_manipulation = nh.advertise<std_msgs::Bool>("/enable_manipulation", 1);
    ros::Subscriber sub_enable = nh.subscribe<std_msgs::Bool>("/enable_manipulation", 1, callback_enable_manipulation);

    check_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

//Hello world and get inputs

    std::cout << "*********************   TIAGO  *********************" << std::endl;
    std::cout << "****************************************************" << std::endl;
    std::cout << "A continuació especifica els medicaments a agafar i el pacient. " <<std::endl;
    std::cout << "                                                    " << std::endl;
    std::cout << "Medicaments disponibles : A, B, C"                    << std::endl;
    std::cout << "Pacients : 1, 2"                                    << std::endl;
    std::cout << "                                                    " << std::endl;
    std::cout << "Medicament nº1 : ";
    std::string medic_1;
    std::getline(std::cin, medic_1);
    std::cout << "Medicament nº2 : ";
    std::string medic_2;
    std::getline(std::cin, medic_2);
    std::cout << "Medicament nº3 : ";
    std::string medic_3;
    std::getline(std::cin, medic_3);
    std::cout << "                                                    " << std::endl;
    std::cout << "Pacient : ";
    std::string pacient;
    std::getline(std::cin, pacient);   
    

    initialize_var();
    
    int count=0;

    while(ros::ok()){

        if(enable_manipulation.data == false){

            if(count == 0){
                // clear coastmap
                std_srvs::Empty emptymsg;
                ros::service::call("/move_base/clear_costmaps",emptymsg);

                // switch medicament 1
                switch(value(medic_1.c_str())){
                    case 'A':
                    ROS_INFO("Taula A - medicament A");
                    goToPosition(tableA);
                    break;
                    case 'B':
                    ROS_INFO("Taula B - medicament B");
                    goToPosition(tableB);
                    break;
                    case 'C':
                    ROS_INFO("Taula C - medicament C");
                    goToPosition(tableC);
                    break;
                    default:
                    ROS_INFO("Sense medicament");
                    break;
                }

                enable_manipulation.data = true;
                pub_enable_manipulation.publish(enable_manipulation);
                ros::Duration(1.0);
                count ++;
                
            }
            else if (count == 1){
                // clear coastmap
                std_srvs::Empty emptymsg;
                ros::service::call("/move_base/clear_costmaps",emptymsg);

                // switch medicament 2
                switch(value(medic_2.c_str())){
                    case 'A':
                    ROS_INFO("Taula A - medicament A");
                    goToPosition(tableA);
                    break;
                    case 'B':
                    ROS_INFO("Taula B - medicament B");
                    //goToPosition(centre);
                    goToPosition(tableB);
                    break;
                    case 'C':
                    ROS_INFO("Taula C - medicament C");
                    goToPosition(tableC);
                    break;
                    default:
                    ROS_INFO("Sense medicament");
                    break;
                }
                
                enable_manipulation.data = true;
                pub_enable_manipulation.publish(enable_manipulation);
                ros::Duration(1.0);
                count ++;
                            }
            else if (count == 2){
                // clear coastmap
                std_srvs::Empty emptymsg;
                ros::service::call("/move_base/clear_costmaps",emptymsg);

                // switch medicament 3
                switch(value(medic_3.c_str())){
                    case 'A':
                    ROS_INFO("Taula A - medicament A");
                    goToPosition(tableA);
                    break;
                    case 'B':
                    ROS_INFO("Taula B - medicament B");
                    //goToPosition(centre);
                    goToPosition(tableB);
                    break;
                    case 'C':
                    ROS_INFO("Taula C - medicament C");
                    goToPosition(tableC);
                    break;
                    default:
                    ROS_INFO("Sense medicament");
                    break;
                }

                enable_manipulation.data = true;
                pub_enable_manipulation.publish(enable_manipulation);
                ros::Duration(1.0);
                count ++;

            }
            else if (count == 3){
                //clear coastmap
                std_srvs::Empty emptymsg;
                ros::service::call("/move_base/clear_costmaps",emptymsg);
                
                // switch pacient
                switch(value(pacient.c_str())){
                    case '1':
                    ROS_INFO("Pacient 1");
                    goToPosition(pacient1);
                    break;
                    case '2':
                    ROS_INFO("Pacient 2");
                    goToPosition(pacient2); 
                }

            //ROS_INFO("Anem a la taula");
            //goToPosition(tableC);
            

            }
        }
        else if (enable_manipulation.data == true){
            //ROS_INFO("Agafem objecte");
        }
        else{
            ROS_INFO("else");
        }

        //pub_enable_manipulation.publish(enable_manipulation);



        ros::spinOnce();

    }

    return 0;


}
