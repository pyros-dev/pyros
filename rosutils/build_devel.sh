#!/usr/bin/env bash
CI_ROS_DISTRO=$1
source /opt/ros/$CI_ROS_DISTRO/setup.bash && \
mkdir build && \
cd build && \
cmake .. -DCMAKE_INSTALL_PREFIX=./install && \
make -j1
