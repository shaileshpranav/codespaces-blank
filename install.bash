#!/bin/bash

sudo apt install -y ros-$ROS_DISTRO-turtlebot3
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-bringup
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-description
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-gazebo
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-msgs
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-navigation
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-simulations
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-slam
sudo apt install -y ros-$ROS_DISTRO-move-base
sudo apt install -y ros-$ROS_DISTRO-move-base-msgs
sudo apt install -y ros-$ROS_DISTRO-aruco-detect
sudo apt install -y ros-$ROS_DISTRO-tf
sudo apt install -y ros-$ROS_DISTRO-tf2-ros
sudo apt install -y ros-$ROS_DISTRO-actionlib
sudo apt install -y ros-$ROS_DISTRO-actionlib-msgs
sudo apt install -y ros-$ROS_DISTRO-amcl
sudo apt install -y ros-$ROS_DISTRO-map-server
sudo apt install -y ros-$ROS_DISTRO-navigation
sudo apt-get install -y ros-$ROS_DISTRO-gazebo-ros-pkgs ros-noetic-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-gazebo-msga
sudo apt-get install -y libgazebo9-dev

source /opt/ros/noetic/setup.bash
rospack profile
sudo rosdep init
rosdep update --include-eol-distros 
