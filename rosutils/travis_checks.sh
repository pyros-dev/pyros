#!/usr/bin/env bash
set -e

# These variables need to be setup before calling this script:
# ROS_FLOW [devel | install]

cd build

if [ "$ROS_FLOW" == "devel" ]; then
    source devel/setup.bash
    echo PYTHONPATH = $PYTHONPATH
    rospack profile
    make -j1 tests
    make -j1 run_tests
    catkin_test_results .
elif [ "$ROS_FLOW" == "install" ]; then
    source install/setup.bash
    echo PYTHONPATH = $PYTHONPATH
    rospack profile
    python -m nose pyros -s
fi
