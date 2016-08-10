#!/usr/bin/env bash

# These variables need to be setup before calling this script:
# CI_ROS_DISTRO [indigo | jade]

set -ex
# Add ROS repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
# Install ROS base
sudo apt-get install ros-$CI_ROS_DISTRO-ros-base -y
# Install and initialize rosdep
sudo apt-get install python-rosdep -y
sudo `which rosdep` init
rosdep update
