#!/usr/bin/env bash

# bootstrap.sh - Shell script for Vagrant to install ROS and additional 
# packages for Rockie.
#
# Bryant Pong
# 7/29/14
# Last Updated: 7/29/14 - 6:26 PM  

# Add ROS Hydro Software Sources:
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list' 

# Initialize software keys for ROS Hydro:
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

apt-get update
apt-get install -y ros-hydro-desktop-full

# Initialize rosdep:
rosdep init
rosdep update

# Source ROS Hydro so ROS utilities are in PATH:
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Acquire rosinstall:
apt-get install python-rosinstall


