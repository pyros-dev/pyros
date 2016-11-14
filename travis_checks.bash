#!/bin/bash
set -e

# first we ensure we change to the directory where this script is.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

# This script run basic checks on this project.
# It is used by travis and can also be used by a developer for checking his current working tree.
#
# These variables need to be setup before calling this script:
# ROS_DISTRO [indigo | jade | kinetic]
# ROS_FLOW [devel | install]

if [ ! -z ${ROS_FLOW+undef} ]; then
# ROS_FLOW ONLY !!

    # For travis docker, this is already done by the entrypoint in docker image.
    # However when using 'docker exec' we still need to source it ourselves.
    # Also it is mandatory when this script is run directly by the developer.
    source /opt/ros/$ROS_DISTRO/setup.bash

    mkdir -p build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=./install
    if [ "$ROS_FLOW" == "devel" ]; then
        make -j1
        source devel/setup.bash
        echo PYTHONPATH = $PYTHONPATH
        rospack profile
        make -j1 tests
        make -j1 run_tests
        catkin_test_results .
    elif [ "$ROS_FLOW" == "install" ]; then
        make -j1 install
        source install/setup.bash
        echo PYTHONPATH = $PYTHONPATH
        rospack profile
    fi
fi

# In ALL Cases
python -m nose pyros.rosinterface.mock.tests -s
python -m nose pyros.rosinterface.tests -s

# Careful with process conflicting here
python -m nose pyros.rosinterface.rostests.testService -s
python -m nose pyros.rosinterface.rostests.testStringPublisher -s
python -m nose pyros.rosinterface.rostests.testStringSubscriber -s
python -m nose pyros.rosinterface.rostests.test_param_if_pool -s
python -m nose pyros.rosinterface.rostests.test_service_if_pool -s
python -m nose pyros.rosinterface.rostests.test_subscriber_if_pool -s
python -m nose pyros.rosinterface.rostests.test_publisher_if_pool -s
python -m nose pyros.rosinterface.rostests.testRosInterface -s

python -m nose pyros.tests -s
