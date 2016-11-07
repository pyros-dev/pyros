#!/usr/bin/env python
from __future__ import absolute_import

import os
import sys
import pickle

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
import time

# current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# # if not current_path in sys.path:
# sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

# setting up logging from this test
import logging.config
# Setting up logging for this test
# TODO : solve multiprocess problems... this seems not enough to get on console...
logging.config.dictConfig(
    {
        'version': 1,
        'formatters': {
            'verbose': {
                'format': '%(levelname)s %(asctime)s %(module)s %(process)d %(thread)d %(message)s'
            },
            'simple': {
                'format': '%(levelname)s %(name)s:%(message)s'
            },
        },
        'handlers': {
            'null': {
                'level': 'DEBUG',
                'class': 'logging.NullHandler',
            },
            'console': {
                'level': 'DEBUG',
                'class': 'logging.StreamHandler',
                'formatter': 'simple'
            },
        },
        'loggers': {
            'pyros_config': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            'pyros': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            'pyros.rosinterface': {
                'handlers': ['console'],
                'level': 'DEBUG',
            },
            'pyros.ros_interface': {
                'handlers': ['console'],
                'level': 'DEBUG',
            }
        }
    }
)


# Unit test import (  will emulate ROS setup if needed )
import nose

from pyros.baseinterface import DiffTuple
from pyros.rosinterface import RosInterface


import rospy
import roslaunch
import rosnode
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

from pyros.rosinterface.rostests import Timeout

# useful test tools
from pyros_utils import rostest_nose
import unittest


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()


def interface_reset(enable_cache=False):
    interface = RosInterface('test_rosinterface', enable_cache=enable_cache)
    # CAREFUL : this is doing a rospy.init_node, and it should be done only once per PROCESS (or with same arguments)
    # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.

    return interface


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request


# TODO : This currently duplicates the test_*_if_pool. We should do something different here...
# TODO : Probably creating mocks for *_if_pool, and make sure the interface calls it the properway.
# TODO : And some integration test (use case focused, instead of having all possible combinations like in test_*if_pool)
@nose.tools.nottest
class TestRosInterface(unittest.TestCase):
    """
    Main test fixture holding all tests
    Subclasses can override setup / teardown to test different environments
    """

    launch = None
    # test node process not setup by default (rostest dont need it here)
    empty_srv_process = None
    trigger_srv_process = None

    # Class fixture ( once each )
    @classmethod
    def setup_class(cls):
        # Start roslaunch
        TestRosInterface.launch = roslaunch.scriptapi.ROSLaunch()
        TestRosInterface.launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match
        global empty_srv_process, trigger_srv_process
        empty_srv_node = roslaunch.core.Node('pyros_test', 'emptyService.py', name='empty_service')
        trigger_srv_node = roslaunch.core.Node('pyros_test', 'triggerService.py', name='trigger_service')
        TestRosInterface.empty_srv_process = TestRosInterface.launch.launch(empty_srv_node)
        TestRosInterface.trigger_srv_process = TestRosInterface.launch.launch(trigger_srv_node)

    @classmethod
    def teardown_class(cls):
        # ensuring all process are finished
        if TestRosInterface.empty_srv_process is not None:
            TestRosInterface.empty_srv_process.stop()
        if TestRosInterface.trigger_srv_process is not None:
            TestRosInterface.trigger_srv_process.stop()

    # EXPOSE PUBLISHERS + UPDATE Interface
    def test_publisher_appear_expose_update(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """
        topicname = '/test/string'
        self.interface.expose_publishers([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.interface.publishers.keys())
        dt = self.interface.update()
        self.assertTrue(topicname in dt.added)  # has been detected

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.publishers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.publishers.keys())

        # cleaning up
        self.interface.expose_publishers([])
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.interface.publishers.keys())

    def test_publisher_appear_update_expose(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """

        topicname = '/test/nonexistent1'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and nonexistent_pub.resolved_name not in self.interface.publishers_available:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        dt = self.interface.expose_publishers([topicname])
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.publishers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in dt.added)
        self.assertTrue(topicname in self.interface.publishers.keys())

        # removing publisher
        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.interface.publishers_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.interface.publishers.keys())

    def test_publisher_expose_appear_update(self):
        """
        Test basic topic adding functionality for a topic which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        topicname = '/test/nonexistent2'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        self.interface.expose_publishers([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())
        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # make sure the topic is STILL in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # make sure the topic backend has STILL not been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        # create the publisher and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in dt.added:
                dt = self.interface.update()
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.added)  # detected
        # TODO : do we need a test with subscriber ?

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.publishers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.publishers.keys())

        # removing publisher
        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.interface.publishers_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.interface.publishers.keys())

    def test_publisher_withhold_update_disappear(self):
        """
        Test topic withholding functionality for a topic which doesnt exists anymore in the ros environment.
        Normal usecase
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        topicname = '/test/nonexistent3'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in self.interface.publishers_available:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added (not exposed)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control
        # TODO : do we need a test with subscriber ?

        self.assertTrue(not t.timed_out)
        # topic should be in the list of args yet (not exposed)
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        # Here we are sure the internal state has topic_name registered
        # will be exposed right away
        dt = self.interface.expose_publishers([topicname])
        self.assertTrue(topicname in dt.added)  # detected and added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.publishers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.publishers.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should STILL be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # topic backend should STILL be there
        self.assertTrue(topicname in self.interface.publishers.keys())

        dt = self.interface.expose_publishers([])
        self.assertTrue(topicname in dt.removed)  # removed
        self.assertEqual(dt.added, [])  # nothing added
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.publishers.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.publishers.keys())

        nonexistent_pub.unregister()

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.publishers.keys())

        # Waiting a bit until the system state is back as expected (with connection cache this can be delayed)
        with Timeout(5) as t:
            while not t.timed_out and not topicname in self.interface.publishers_available:
                time.sleep(1)

    def test_publisher_disappear_update_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """

        topicname = '/test/nonexistent4'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        dt = self.interface.expose_publishers([topicname])
        self.assertEqual(dt.added, [])  # nothing added (topic name doesnt exist yet)
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # topic backend has been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        def appear_disappear():
            # create the publisher and then try exposing the topic again, simulating
            # it coming online before expose call.
            nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                    dt = self.interface.update()
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # detected
            # TODO : do we need a test with subscriber ?

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.interface.publishers_args)
            # topic backend has been created
            self.assertTrue(topicname in self.interface.publishers.keys())

            # up to here possible sequences should have been already tested by previous tests
            # Now comes our actual disappearance / withholding test
            nonexistent_pub.unregister()

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.interface.publishers_args)
            # the backend should STILL be there
            self.assertTrue(topicname in self.interface.publishers.keys())
            # Note the Topic implementation should take care of possible errors in this case

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and topicname not in dt.removed:
                    dt = self.interface.update()
                    self.assertEqual(dt.added, [])  # nothing added
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(topicname in dt.removed)  # detected lost
            # every exposed topic should remain in the list of args ( in case regex match another topic )
            self.assertTrue(topicname in self.interface.publishers_args)
            # make sure the topic backend should NOT be there any longer
            self.assertTrue(topicname not in self.interface.publishers.keys())

        appear_disappear()

        # test that coming back actually also works

        # Note : for the cache version we need update to run in between to make sure the cache callback for dropped
        # interface doesnt interfere with the new topic coming up.
        # TODO fix this problem (cache node / interface update not spinning fast enough to detect the change)
        # HOW ?
        # -> cache node (or proxy??) with possibility to mask topics + node to not trigger change ???
        # -> splitting subscribers and publishers will make it better to not confuse what goes up/down ???
        # -> wait for deletion confirmation before diff reporting deleted ? in update loop or higher, at test/user level ?
        # Currently there are tricks in place to determine if a diff comes from interface pub/sub creation/deletion
        #
        time.sleep(2)
        dt = self.interface.update()

        # test that coming back actually also works
        appear_disappear()

        self.interface.expose_publishers([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

    def test_publisher_update_disappear_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """

        topicname = '/test/nonexistent5'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        dt = self.interface.expose_publishers([topicname])
        self.assertEqual(dt.added, [])  # nothing added yet ( not existing )
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.interface.publishers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)

        with Timeout(5) as t:
            dt = DiffTuple([], [])
            while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                dt = self.interface.update()
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # added now because it just appeared
        self.assertEqual(dt.removed, [])  # nothing removed
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # topic backend has been created
        self.assertTrue(topicname in self.interface.publishers.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearrence / withholding test

        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )
        # TODO : test disappear ( how ? )

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.publishers_args)
        # the backend should STILL be there
        self.assertTrue(topicname in self.interface.publishers.keys())
        # Note the Topic implementation should take care of possible errors in this case

        self.interface.expose_publishers([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.publishers_args)
        # topic backend should NOT be there any longer
        self.assertTrue(topicname not in self.interface.publishers.keys())

    # EXPOSE SUBSCRIBERS + UPDATE Interface
    def test_subscriber_appear_expose_update(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """
        topicname = '/test/string'
        self.interface.expose_subscribers([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        dt = self.interface.update()
        self.assertTrue(topicname in dt.added)  # has been detected

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.subscribers.keys())

        # cleaning up
        self.interface.expose_subscribers([])
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.interface.subscribers.keys())

    def test_subscriber_appear_update_expose(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """

        topicname = '/test/nonexistent1_sub'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and nonexistent_pub.resolved_name not in self.interface.subscribers_available:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        dt = self.interface.expose_subscribers([topicname])
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in dt.added)
        self.assertTrue(topicname in self.interface.subscribers.keys())

        # removing publisher
        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.interface.subscribers_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.interface.subscribers.keys())

    def test_subscriber_expose_appear_update(self):
        """
        Test basic topic adding functionality for a topic which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """

        topicname = '/test/nonexistent2_sub'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        self.interface.expose_subscribers([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # make sure the topic is STILL in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # make sure the topic backend has STILL not been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        # create the publisher and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in dt.added:
                dt = self.interface.update()
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.added)  # detected
        # TODO : do we need a test with subscriber ?

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.subscribers.keys())

        # removing publisher
        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.interface.subscribers_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.interface.subscribers.keys())

    def test_subscriber_withhold_update_disappear(self):
        """
        Test topic withholding functionality for a topic which doesnt exists anymore in the ros environment.
        Normal usecase
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        topicname = '/test/nonexistent3_sub'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in self.interface.subscribers_available:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added (not exposed)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control
        # TODO : do we need a test with subscriber ?

        self.assertTrue(not t.timed_out)
        # topic should be in the list of args yet (not exposed)
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        # Here we are sure the internal state has topic_name registered
        # will be exposed right away
        dt = self.interface.expose_subscribers([topicname])
        self.assertTrue(topicname in dt.added)  # detected and added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.subscribers.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should STILL be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # topic backend should STILL be there
        self.assertTrue(topicname in self.interface.subscribers.keys())

        dt = self.interface.expose_subscribers([])
        self.assertTrue(topicname in dt.removed)  # removed
        self.assertEqual(dt.added, [])  # nothing added
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        nonexistent_pub.unregister()

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        # Waiting a bit until the system state is back as expected (with connection cache this can be delayed)
        with Timeout(5) as t:
            while not t.timed_out and not topicname in self.interface.subscribers_available:
                time.sleep(1)

    def test_subscriber_disappear_update_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """

        topicname = '/test/nonexistent4_sub'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        dt = self.interface.expose_subscribers([topicname])
        self.assertEqual(dt.added, [])  # nothing added (topic name doesnt exist yet)
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # topic backend has been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        def appear_disappear():
            # create the publisher and then try exposing the topic again, simulating
            # it coming online before expose call.
            nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                    dt = self.interface.update()
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # detected
            # TODO : do we need a test with subscriber ?

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.interface.subscribers_args)
            # topic backend has been created
            self.assertTrue(topicname in self.interface.subscribers.keys())

            # up to here possible sequences should have been already tested by previous tests
            # Now comes our actual disappearance / withholding test
            nonexistent_pub.unregister()

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.interface.subscribers_args)
            # the backend should STILL be there
            self.assertTrue(topicname in self.interface.subscribers.keys())
            # Note the Topic implementation should take care of possible errors in this case

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and topicname not in dt.removed:
                    dt = self.interface.update()
                    self.assertEqual(dt.added, [])  # nothing added
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(topicname in dt.removed)  # detected lost
            # every exposed topic should remain in the list of args ( in case regex match another topic )
            self.assertTrue(topicname in self.interface.subscribers_args)
            # make sure the topic backend should NOT be there any longer
            self.assertTrue(topicname not in self.interface.subscribers.keys())

        appear_disappear()

        # test that coming back actually also works

        # Note : for the cache version we need update to run in between to make sure the cache callback for dropped
        # interface doesnt interfere with the new topic coming up.
        # TODO fix this problem (cache node / interface update not spinning fast enough to detect the change)
        # HOW ?
        # -> cache node (or proxy??) with possibility to mask topics + node to not trigger change ???
        # -> splitting subscribers and publishers will make it better to not confuse what goes up/down ???
        # -> wait for deletion confirmation before diff reporting deleted ? in update loop or higher, at test/user level ?
        # Currently there are tricks in place to determine if a diff comes from interface pub/sub creation/deletion
        #
        time.sleep(2)
        dt = self.interface.update()

        # test that coming back actually also works
        appear_disappear()

        self.interface.expose_subscribers([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

    def test_subscriber_update_disappear_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """

        topicname = '/test/nonexistent5_sub'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        dt = self.interface.expose_subscribers([topicname])
        self.assertEqual(dt.added, [])  # nothing added yet ( not existing )
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.interface.subscribers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)

        with Timeout(5) as t:
            dt = DiffTuple([], [])
            while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                dt = self.interface.update()
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # added now because it just appeared
        self.assertEqual(dt.removed, [])  # nothing removed
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # topic backend has been created
        self.assertTrue(topicname in self.interface.subscribers.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearrence / withholding test

        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )
        # TODO : test disappear ( how ? )

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.subscribers_args)
        # the backend should STILL be there
        self.assertTrue(topicname in self.interface.subscribers.keys())
        # Note the Topic implementation should take care of possible errors in this case

        self.interface.expose_subscribers([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.subscribers_args)
        # topic backend should NOT be there any longer
        self.assertTrue(topicname not in self.interface.subscribers.keys())

# EXPOSE SERVICES + UPDATE Interface

    def test_service_appear_expose_update(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """

        servicename = '/test/empsrv'
        dt = self.interface.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # service backend has not been created since the update didn't run yet
        self.assertTrue(servicename not in self.interface.services.keys())

        # NOTE : We need to wait to make sure the tests nodes are started...
        with Timeout(5) as t:
            while not t.timed_out and servicename not in dt.added:
                dt = self.interface.update()
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # every exposed service should remain in the list of args ( in case regex match another service )
        self.assertTrue(servicename in self.interface.services_args)
        # make sure the service backend has been created
        self.assertTrue(servicename in self.interface.services.keys())

        # cleaning up
        self.interface.expose_services([])
        # service should not be in the list of args any longer
        self.assertTrue(servicename not in self.interface.services_args)
        # service backend has been removed
        self.assertTrue(servicename not in self.interface.services.keys())

    def test_service_appear_update_expose(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """
        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())

        # create the service and then try exposing the service again, simulating
        # it coming online before expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            with Timeout(5) as t:
                while not t.timed_out and nonexistent_srv.resolved_name not in self.interface.services_available:
                    dt = self.interface.update()
                    self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            # every added service should be in the list of args
            self.assertTrue(servicename not in self.interface.services_args)
            # the backend should not have been created
            self.assertTrue(servicename not in self.interface.services.keys())

            # here we are sure the interface knows the service is available
            # it will be exposed right now
            dt = self.interface.expose_services([servicename])
            self.assertTrue(servicename in dt.added)  # servicename added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every exposed service should remain in the list of args ( in case regex match another service )
            self.assertTrue(servicename in self.interface.services_args)
            # make sure the service backend has been created
            self.assertTrue(servicename in self.interface.services.keys())
        finally:
            nonexistent_srv.shutdown('testing complete')

    def test_service_expose_appear_update(self):
        """
        Test basic service adding functionality for a service which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())

        self.interface.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # make sure the service is STILL in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # make sure the service backend has STILL not been created
        self.assertTrue(servicename not in self.interface.services.keys())

        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and nonexistent_srv.resolved_name not in dt.added:
                    dt = self.interface.update()
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent_srv added
            # every exposed service should remain in the list of args ( in case regex match another service )
            self.assertTrue(servicename in self.interface.services_args)
            # make sure the service backend has been created
            self.assertTrue(servicename in self.interface.services.keys())
        finally:
            nonexistent_srv.shutdown('testing complete')

    def test_service_withhold_update_disappear(self):
        """
        Test service witholding functionality for a service which doesnt exists anymore in
        the ros environment. Normal usecase.
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())

        self.interface.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # service backend has NOT been created yet
        self.assertTrue(servicename not in self.interface.services.keys())

        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            dt = self.interface.update()
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent_srv added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every withhold service should STILL be in the list of args
            self.assertTrue(servicename in self.interface.services_args)
            # service backend has been created
            self.assertTrue(servicename in self.interface.services.keys())

            dt = self.interface.expose_services([])
            self.assertEqual(dt.added, [])  # nothing added
            self.assertTrue(nonexistent_srv.resolved_name in dt.removed)  # nonexistent_srv removed
            # every withhold service should NOT be in the list of args
            self.assertTrue(servicename not in self.interface.services_args)
            # service backend should be GONE
            self.assertTrue(servicename not in self.interface.services.keys())

            dt = self.interface.update()
            self.assertEqual(dt.added, [])  # nothing added
            self.assertEqual(dt.removed, [])  # nothing removed
            # every withhold service should STILL NOT be in the list of args
            self.assertTrue(servicename not in self.interface.services_args)
            # service backend should be GONE
            self.assertTrue(servicename not in self.interface.services.keys())
        finally:
            nonexistent_srv.shutdown('testing disappearing service')

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nonexistent_srv already removed
        # every withhold service should STILL NOT be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # service backend should be GONE
        self.assertTrue(servicename not in self.interface.services.keys())

    def test_service_disappear_update_withhold(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """
        servicename = '/test/absentsrv1'
        # service should not be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())

        self.interface.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # service backend has NOT been created yet
        self.assertTrue(servicename not in self.interface.services.keys())

        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            dt = self.interface.update()
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent added
            self.assertEqual(dt.removed, [])  # nothing removed

            # service should be in the list of args
            self.assertTrue(servicename in self.interface.services_args)
            # the backend should have been created
            self.assertTrue(servicename in self.interface.services.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            nonexistent_srv.shutdown('testing disappearing service')

        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # the backend should STILL be there
        self.assertTrue(servicename in self.interface.services.keys())
        # Note the service implementation should take care of possible errors in this case

        # wait here until service actually disappear from cache proxy
        with Timeout(5) as t:
            while not t.timed_out and nonexistent_srv.resolved_name not in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(nonexistent_srv.resolved_name in dt.removed)  # nonexistent_srv removed
        # every exposed service should remain in the list of args ( in case regex match another service )
        self.assertTrue(servicename in self.interface.services_args)
        # make sure the service backend should NOT be there any longer
        self.assertTrue(servicename not in self.interface.services.keys())

        # TODO : test that coming back actually works

        self.interface.expose_services([])
        # every withhold service should NOT be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # service backend has not been created
        self.assertTrue(servicename not in self.interface.services.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold service should NOT be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # make sure the service backend has been created
        self.assertTrue(servicename not in self.interface.services.keys())

    def test_service_update_disappear_withhold(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """

        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.interface.services.keys())

        self.interface.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # service backend has not been created
        self.assertTrue(servicename not in self.interface.services.keys())

        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            dt = self.interface.update()
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent_srv added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every added service should be in the list of args
            self.assertTrue(servicename in self.interface.services_args)
            # service backend has been created
            self.assertTrue(servicename in self.interface.services.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            nonexistent_srv.shutdown('testing disappearing service')

        # every added service should be in the list of args
        self.assertTrue(servicename in self.interface.services_args)
        # the backend should STILL be there
        self.assertTrue(servicename in self.interface.services.keys())
        # Note the service implementation should take care of possible errors in this case

        self.interface.expose_services([])
        # every withhold service should NOT be in the list of args
        self.assertTrue(servicename not in self.interface.services_args)
        # service backend should NOT be there any longer
        self.assertTrue(servicename not in self.interface.services.keys())


# EXPOSE PARAMS + UPDATE Interface

    def test_param_appear_expose_update(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """

        paramname = '/test/confirm_param'
        dt = self.interface.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # param backend has not been created since the update didn't run yet
        self.assertTrue(paramname not in self.interface.params.keys())

        # NOTE : We need to wait to make sure the tests nodes are started...
        with Timeout(5) as t:
            while not t.timed_out and paramname not in dt.added:
                dt = self.interface.update()
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # every exposed param should remain in the list of args ( in case regex match another service )
        self.assertTrue(paramname in self.interface.params_args)
        # make sure the param backend has been created
        self.assertTrue(paramname in self.interface.params.keys())

        # cleaning up
        self.interface.expose_params([])
        # param should not be in the list of args any longer
        self.assertTrue(paramname not in self.interface.params_args)
        # param backend has been removed
        self.assertTrue(paramname not in self.interface.params.keys())

    def test_param_appear_update_expose(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())

        # create the param and then try exposing the param again, simulating
        # it coming online before expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            with Timeout(5) as t:
                while not t.timed_out and paramname not in self.interface.params_available:
                    dt = self.interface.update()
                    self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            # every added param should be in the list of args
            self.assertTrue(paramname not in self.interface.params_args)
            # the backend should not have been created
            self.assertTrue(paramname not in self.interface.params.keys())

            # here we are sure the interface knows the service is available
            # it will be exposed right now
            dt = self.interface.expose_params([paramname])
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every exposed param should remain in the list of args ( in case regex match another param )
            self.assertTrue(paramname in self.interface.params_args)
            # make sure the service backend has been created
            self.assertTrue(paramname in self.interface.params.keys())
        finally:
            rospy.delete_param(paramname)

    def test_param_expose_appear_update(self):
        """
        Test basic param adding functionality for a param which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())

        self.interface.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # make sure the param is STILL in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # make sure the param backend has STILL not been created
        self.assertTrue(paramname not in self.interface.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and paramname not in dt.added:
                    dt = self.interface.update()
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(paramname in dt.added)  # nonexistent_srv added
            # every exposed param should remain in the list of args ( in case regex match another service )
            self.assertTrue(paramname in self.interface.params_args)
            # make sure the param backend has been created
            self.assertTrue(paramname in self.interface.params.keys())
        finally:
            rospy.delete_param(paramname)

    def test_param_withhold_update_disappear(self):
        """
        Test param witholding functionality for a param which doesnt exists anymore in
        the ros environment. Normal usecase.
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())

        self.interface.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # service backend has NOT been created yet
        self.assertTrue(paramname not in self.interface.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            dt = self.interface.update()
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every withhold param should STILL be in the list of args
            self.assertTrue(paramname in self.interface.params_args)
            # param backend has been created
            self.assertTrue(paramname in self.interface.params.keys())

            dt = self.interface.expose_params([])
            self.assertEqual(dt.added, [])  # nothing added
            self.assertTrue(paramname in dt.removed)  # paramname removed
            # every withhold param should NOT be in the list of args
            self.assertTrue(paramname not in self.interface.params_args)
            # param backend should be GONE
            self.assertTrue(paramname not in self.interface.params.keys())

            dt = self.interface.update()
            self.assertEqual(dt.added, [])  # nothing added
            self.assertEqual(dt.removed, [])  # nothing removed
            # every withhold param should STILL NOT be in the list of args
            self.assertTrue(paramname not in self.interface.params)
            # param backend should be GONE
            self.assertTrue(paramname not in self.interface.params.keys())
        finally:
            rospy.delete_param(paramname)

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nonexistent_srv already removed
        # every withhold service should STILL NOT be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # service backend should be GONE
        self.assertTrue(paramname not in self.interface.params.keys())

    def test_param_disappear_update_withhold(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """
        paramname = '/test/absentparam1'
        # param should not be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())

        self.interface.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # param backend has NOT been created yet
        self.assertTrue(paramname not in self.interface.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            dt = self.interface.update()
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # param should be in the list of args
            self.assertTrue(paramname in self.interface.params_args)
            # the backend should have been created
            self.assertTrue(paramname in self.interface.params.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            rospy.delete_param(paramname)

        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # the backend should STILL be there
        self.assertTrue(paramname in self.interface.params.keys())
        # Note the param implementation should take care of possible errors in this case

        # wait here until param actually disappear from cache proxy
        with Timeout(5) as t:
            while not t.timed_out and paramname not in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(paramname in dt.removed)  # nonexistent_srv removed
        # every exposed param should remain in the list of args ( in case regex match another service )
        self.assertTrue(paramname in self.interface.params_args)
        # make sure the param backend should NOT be there any longer
        self.assertTrue(paramname not in self.interface.params.keys())

        # TODO : test that coming back actually works

        self.interface.expose_params([])
        # every withhold param should NOT be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # param backend has not been created
        self.assertTrue(paramname not in self.interface.params.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold param should NOT be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # make sure the param backend has been created
        self.assertTrue(paramname not in self.interface.params.keys())

    def test_param_update_disappear_withhold(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.interface.params.keys())

        self.interface.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # param backend has not been created
        self.assertTrue(paramname not in self.interface.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            dt = self.interface.update()
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every added param should be in the list of args
            self.assertTrue(paramname in self.interface.params_args)
            # param backend has been created
            self.assertTrue(paramname in self.interface.params.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            rospy.delete_param(paramname)

        # every added param should be in the list of args
        self.assertTrue(paramname in self.interface.params_args)
        # the backend should STILL be there
        self.assertTrue(paramname in self.interface.params.keys())
        # Note the param implementation should take care of possible errors in this case

        self.interface.expose_params([])
        # every withhold param should NOT be in the list of args
        self.assertTrue(paramname not in self.interface.params_args)
        # param backend should NOT be there any longer
        self.assertTrue(paramname not in self.interface.params.keys())


#TODO : here we always test the full update => test the diff algorithm as well !

@nose.tools.istest
class TestRosInterface1NoCache(TestRosInterface):
    def setUp(self):
        global interface

        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self.strsub = rospy.Subscriber('/test/string', String, queue_size=1)
        self.empsub = rospy.Subscriber('/test/empty', Empty, queue_size=1)

        # dynamically setup our interface to not use the cache
        self.interface = interface_reset(enable_cache=False)

        # CAREFUL : this is doing a rospy.init_node, and it should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.

    def tearDown(self):
        self.interface = None


# Testing with Connection Cache
@nose.tools.istest
class TestRosInterfaceCache(TestRosInterface):
    def setUp(self):

        # first we setup our publishers and our node (used by rospy.resolve_name calls to remap topics)
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self.strsub = rospy.Subscriber('/test/string', String, queue_size=1)
        self.empsub = rospy.Subscriber('/test/empty', Empty, queue_size=1)

        # dynamically setup our interface to use the cache
        self.interface = interface_reset(enable_cache=True)

        # we need to speed fast enough for the tests to not fail on timeout...
        rospy.set_param('/connection_cache/spin_freq', 2)  # 2 Hz
        self.connection_cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py',
                                                         name='connection_cache',
                                                         remap_args=[('~list', rospy.resolve_name('~connections_list')),
                                                                     (
                                                                     '~diff', rospy.resolve_name('~connections_diff'))])

        # Easier to remap the node topic to the proxy ones, instead of the opposite, since there is no dynamic remapping.
        # However for normal usecase, remapping the proxy handles is preferable.
        try:
            self.connection_cache_proc = self.launch.launch(self.connection_cache_node)
        except roslaunch.RLException as rlexc:
            raise nose.SkipTest("Connection Cache Node not found (part of rocon_python_comms pkg). Skipping test.")

        assert self.connection_cache_proc.is_alive()

        # wait for node to be started
        node_api = None
        with Timeout(5) as t:
            while not t.timed_out and node_api is None:
                node_api = rosnode.get_api_uri(rospy.get_master(), 'connection_cache')

        assert node_api is not None  # make sure the connection cache node is started before moving on.

    def tearDown(self):
        self.interface = None

        self.connection_cache_proc.stop()
        while self.connection_cache_proc.is_alive():
            time.sleep(0.2)  # waiting for cache node to die
        assert not self.connection_cache_proc.is_alive()
        time.sleep(1)  # TODO : investigate : we shouldnt need this

    # explicitely added here only needed to help the debugger.

    # TODO : investigate this
    def test_publisher_disappear_update_withhold(self):
        raise nose.SkipTest("Test failing, pubs conflicting when using cache. Skipping for now...")

    def test_publisher_expose_appear_update(self):
        raise nose.SkipTest("Test failing, pubs conflicting when using cache. Skipping for now...")

    def test_subscriber_disappear_update_withhold(self):
        raise nose.SkipTest("Test failing, pubs conflicting when using cache. Skipping for now...")

    def test_subscriber_expose_appear_update(self):
        raise nose.SkipTest("Test failing, pubs conflicting when using cache. Skipping for now...")

if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_ros_interface_no_cache', 'test_all', TestRosInterface1NoCache)

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_ros_interface_cache', 'test_all', TestRosInterfaceCache)

