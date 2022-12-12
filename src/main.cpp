/**
 * @file main.cpp
 * @author Shailesh Pranav Rajendran (spraj@umd.edu)
 * @brief The script for main class 
 * @version 0.1
 * @date 2022-12-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/wall-e/navigation.hpp"
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <std_msgs/String.h>
#include <iostream>
#include <array>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal explorer_goal[5];

int main(int argc, char** argv)
{


  ros::init(argc, argv, "simple_navigation_goals");   //simple_navigation_goals node starts
  ros::NodeHandle nh;

  // ros::Publisher pubex = nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);   //robot rotation when in goal

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }
  std::array<XmlRpc::XmlRpcValue,5> ex_goal_pos;


  for(int i=0; i<=4;i++)
  {
    explorer_goal[i].target_pose.header.frame_id = "map";
    explorer_goal[i].target_pose.header.stamp = ros::Time::now();
    explorer_goal[i].target_pose.pose.orientation.w = 1.0;
    explorer_goal[i].target_pose.pose.position.x = 3;//ex_goal_pos[i][0];
    explorer_goal[i].target_pose.pose.position.y = 3;//x_goal_pos[i][1];
    // ROS_INFO_STREAM("Explorer: "<<ex_goal_pos[i][0]<<","<<ex_goal_pos[i][1]);
  }



  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
  int cnt_ex = 0;   //Number of goals visited by explorer
  int cnt_fl = 0;
  bool test{true};
  int ti = 0;
  while (ros::ok()) {
     if(cnt_ex<5)
    {
        std::cout<<"Sending goal";
        ROS_INFO("Sending goal for explorer");
        explorer_client.sendGoal(explorer_goal[cnt_ex]);      //goal sent to explorer
        ros::spinOnce();
    }
    cnt_ex++;
      // geometry_msgs::Twist msg;
      // msg.angular.z = 0.2;
      // pubex.publish(msg);
      loop_rate.sleep();

      

      if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
      ros::spinOnce();
      ROS_INFO("Hooray, Explorer reached goal");
      ros::shutdown();
    }
}    
}
    


