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

    python -m nose pyros.mockinterface -s
    python -m nose pyros.rosinterface.tests -s

    # Careful with process conflicting here
    python -m nose pyros.rosinterface.rostests.testService -s
    python -m nose pyros.rosinterface.rostests.testStringTopic -s
    python -m nose pyros.rosinterface.rostests.testRosInterface -s
    python -m nose pyros.rosinterface.rostests.testRosInterfaceCache -s

    python -m nose pyros.tests -s

fi
