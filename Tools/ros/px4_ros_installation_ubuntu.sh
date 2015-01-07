#!/bin/sh

# main ROS Setup
# following http://wiki.ros.org/indigo/Installation/Ubuntu
# run this file with . ./px4_ros_setup_ubuntu.sh

## add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

## add key
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | \
    sudo apt-key add -

## Install main ROS pacakges
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update

## Setup environment variables
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

# get rosinstall
sudo apt-get install python-rosinstall

# Hector packages
sudo apt-get install ros-indigo-hector-quadrotor \
    ros-indigo-octomap-msgs \
    ros-indigo-hector-gazebo

