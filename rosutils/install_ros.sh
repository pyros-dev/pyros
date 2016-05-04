#!/usr/bin/env bash

set -x
CI_ROS_DISTRO=$1
PKG_PATH=$2
echo $CI_ROS_DISTRO
# Add ROS repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
# Install and initialize rosdep
sudo apt-get install python-rosdep -y
sudo `which rosdep` init
rosdep update
# Use rosdep to install current package dependencies
rosdep install --default-yes --from-paths $PKG_PATH --rosdistro $CI_ROS_DISTRO
