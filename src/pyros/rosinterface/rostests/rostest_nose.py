#!/usr/bin/env python
# A very special ROS hack that emulate a ros environment when imported from python
# Useful for using all python tools ( tests, IDE, etc. ) without having to do all the ROS setup beforehand
from __future__ import absolute_import

import sys
import os
import logging
import nose
import collections
import multiprocessing
import time

# ROS Import fix for tests and debugging should be done in __init__ of rosinterface
import rospy
import rosgraph
import roslaunch
import rostest


rostest_enabled = False  # default for python or nose runs

roscore_process = None  # default : we dont need to run roscore


# This should have the same effect as launching the <name>.test file for rostest ( before interpreting the contents )
# Should be used only by nose ( or other python test tool )
def rostest_nose_setup_module():
    if not rostest_enabled:
        if not rosgraph.masterapi.is_online():
            global roscore_process
            # Trying to solve this : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
            def ros_core_launch():
                roslaunch.main(['roscore', '--core'])  # same as rostest_main implementation

            roscore_process = multiprocessing.Process(target=ros_core_launch)
            roscore_process.start()

        # Workaround until https://github.com/ros/ros_comm/pull/711 is merged and released
        time.sleep(2)


def rostest_nose_teardown_module():
    if not rostest_enabled:
        # finishing all process
        if roscore_process is not None:
            roscore_process.terminate()  # make sure everything is stopped


def is_rostest_enabled():
    return rostest_enabled


def rostest_or_nose_main(package, test_name, test, sysargv=sys.argv):
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )

    # Ros arguments will tell us if we started from ros, or from straight python
    rosargs = [arg for arg in sysargv if arg.startswith("__")]

    if len(rosargs) > 0:
        global rostest_enabled
        rostest_enabled = True
        rostest.rosrun(package, test_name, test, sysargv)
    else:
        # if python => run with nose => import will be managed by nose import ( from source )
        nose.runmodule()
