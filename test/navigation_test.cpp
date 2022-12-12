/**
 * @file navigation_test.cpp
 * @author Shailesh Pranav Rajendran (spraj@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include "navigation/navigation.hpp"

// Global variables
bool message_received;
geometry_msgs::PoseStamped pose_message;
geometry_msgs::Twist velocity_message;

bool wait_for_message(const bool& message_received, double timeout = 5) {
    ros::Time start = ros::Time::now();
    ros::Time now;
    while (!message_received) {
        ros::spinOnce();
        now = ros::Time::now();
        if ((now - start).toSec() > timeout) {
            return false;
        }
    }
    return true;
}

void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    message_received = true;
    pose_message.header = msg->header;
    pose_message.pose = msg->pose;
}

TEST(test_navigation_class, test_bin_goal_setting) {
    // Arrange
    message_received = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10,
                                                            pose_callback);
    geometry_msgs::PoseStamped expected_msg;
    expected_msg.pose.orientation.w = 1;
    expected_msg.header.frame_id = "map";

    // Act
    Navigation navigator(&nh);
    navigator.set_bin_location_as_goal();

    // Assert
    ASSERT_TRUE(wait_for_message(message_received, 3));
    EXPECT_EQ(pose_message.pose.orientation, expected_msg.pose.orientation);
    EXPECT_EQ(pose_message.pose.position, expected_msg.pose.position);
    EXPECT_EQ(pose_message.header.frame_id, expected_msg.header.frame_id);
}
