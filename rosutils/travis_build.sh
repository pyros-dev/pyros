#!/usr/bin/env bash
set -e

# These variables need to be setup before calling this script:
# CI_ROS_DISTRO [indigo | jade]
# ROS_FLOW [devel | install]

source /opt/ros/$CI_ROS_DISTRO/setup.bash
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=./install

if [ "$ROS_FLOW" == "devel" ]; then
    make -j1
elif [ "$ROS_FLOW" == "install" ]; then
    make -j1 install
fi
