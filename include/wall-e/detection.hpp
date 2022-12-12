
/**
 * @file detection.hpp
 * @author Aman Sharma (amankrsharma3@gmail.com)
 * @brief The header file for the detection part of the Robot.
 * @version 0.1
 * @date 2022-12-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
// ROS headers
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

// Std C++ headers
#include <string.h>
#include <vector>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class DetectObject {
 public:
   /**
    * @brief Constructor for the DetectObject class
    * 
    * @param node_handle 
    */
    explicit DetectObject(ros::NodeHandle*);

    /**
     * @brief Method to detect pre-defined object
     * 
     * @return true if the object is detected
     * @return false  if the object is not detected 
     */
    bool detect_object();

    /**
     * @brief Flag to check if the object is detected
     */
    bool is_object_detected;

    /**
   * @brief callback function for receiving image published by Tiago
   * 
   * @param msg 
   */
    void image_cb(const sensor_msgs::ImageConstPtr&);

 private:
    ros::NodeHandle* nh_;
    cv::Mat img_bgr_, frame_hsv_, frame_thresh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_;
    std::vector<std::vector<cv::Point>> contours_;
};