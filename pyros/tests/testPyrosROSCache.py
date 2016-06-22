#!/usr/bin/env python
from __future__ import absolute_import

import sys
import os
import logging


# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

# Unit test import (  will emulate ROS setup if needed )
import time

from pyros import PyrosROS

# we import pyros.rosinterface here only to get the ROs setup done if needed,
# because we need to use ros modules for testing here
import pyros.rosinterface

import rospy
import roslaunch
import rosnode
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

import pyzmp

from pyros.rosinterface.rostests import Timeout
from pyros.tests.testPyrosROS import TestPyrosROS

# useful test tools
from pyros_setup import rostest_nose
import unittest
import nose
from nose.tools import assert_true, assert_equal, timed

# test node process not setup by default (rostest dont need it here)
launch = None


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        global launch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # Note : rospy.init_node is forbidden here because it would prevent the PyrosROS Process to start()
        # Also we cannot use rospy.init_node in more than 1 test here


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request


# Testing with Connection Cache
@nose.tools.istest
class TestPyrosROSCache(TestPyrosROS):

    def setUp(self):
        # we need to speed fast enough for the tests to not fail on timeout...
        rospy.set_param('/connection_cache/spin_freq', 2)  # 2 Hz
        self.connection_cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py', name='connection_cache',
                                                         remap_args=[('~list', '/pyros_ros/connections_list'),
                                                                     ('~diff', '/pyros_ros/connections_diff'),
                                                                     ])
        try:
            self.connection_cache_proc = self.launch.launch(self.connection_cache_node)
        except roslaunch.RLException as rlexc:
            raise nose.SkipTest("Connection Cache Node not found (part of rocon_python_comms pkg). Skipping test.")

        node_api = None
        with Timeout(5) as t:
            while not t.timed_out and node_api is None:
                node_api = rosnode.get_api_uri(rospy.get_master(), 'connection_cache')

        assert node_api is not None  # make sure the connection cache node is started before moving on.

        super(TestPyrosROSCache, self).setUp(enable_cache=True)

    def tearDown(self):
        super(TestPyrosROSCache, self).tearDown()

        self.connection_cache_proc.stop()
        while self.connection_cache_proc.is_alive():
            time.sleep(0.2)  # waiting for cache node to die
        assert not self.connection_cache_proc.is_alive()
        time.sleep(1)  # TODO : investigate : we shouldnt need this


if __name__ == '__main__':

    import nose
    nose.runmodule()




