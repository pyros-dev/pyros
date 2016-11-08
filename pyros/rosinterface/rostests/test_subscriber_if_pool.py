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
from pyros.rosinterface import RosSubscriberIfPool
from pyros.rosinterface.connection_cache_utils import connection_cache_proxy_create, connection_cache_marshall


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

    # Using pubs and subs require us to be a node
    rospy.init_node('test_topic_if_pool', argv=None, disable_signals=True)
    # CAREFUL : rospy.init_node should be done only once per PROCESS
    # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


@nose.tools.nottest
class TestRosSubscriberIfPool(unittest.TestCase):
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
        TestRosSubscriberIfPool.launch = roslaunch.scriptapi.ROSLaunch()
        TestRosSubscriberIfPool.launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match
        global empty_srv_process, trigger_srv_process
        empty_srv_node = roslaunch.core.Node('pyros_test', 'emptyService.py', name='empty_service')
        trigger_srv_node = roslaunch.core.Node('pyros_test', 'triggerService.py', name='trigger_service')
        TestRosSubscriberIfPool.empty_srv_process = TestRosSubscriberIfPool.launch.launch(empty_srv_node)
        TestRosSubscriberIfPool.trigger_srv_process = TestRosSubscriberIfPool.launch.launch(trigger_srv_node)

    @classmethod
    def teardown_class(cls):
        # ensuring all process are finished
        if TestRosSubscriberIfPool.empty_srv_process is not None:
            TestRosSubscriberIfPool.empty_srv_process.stop()
        if TestRosSubscriberIfPool.trigger_srv_process is not None:
            TestRosSubscriberIfPool.trigger_srv_process.stop()

    # EXPOSE SUBSCRIBERS + UPDATE Interface
    def test_subscriber_appear_expose_update(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """
        topicname = '/test/string'
        self.subscriber_if_pool.expose_subscribers([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        dt = DiffTuple([], [])
        # NOTE : We need to wait to make sure the tests nodes are started...
        with Timeout(5) as t:
            while not t.timed_out and topicname not in dt.added:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.added)  # has been detected

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

        # cleaning up
        self.subscriber_if_pool.expose_subscribers([])
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

    def test_subscriber_appear_update_expose(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """

        topicname = '/test/nonexistent1'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())
        # First update should not change state
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and nonexistent_pub.resolved_name not in self.subscriber_if_pool.subscribers_available:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        dt = self.subscriber_if_pool.expose_subscribers([topicname])
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in dt.added)
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

        # removing publisher
        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

    def test_subscriber_expose_appear_update(self):
        """
        Test basic topic adding functionality for a topic which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        topicname = '/test/nonexistent2'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())
        # First update should not change state
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertTrue(topicname not in dt.added)  # not detected
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        self.subscriber_if_pool.expose_subscribers([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertTrue(topicname not in dt.added)  # not detected
        # make sure the topic is STILL in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # make sure the topic backend has STILL not been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        # create the publisher and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in dt.added:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.added)  # detected
        # TODO : do we need a test with subscriber ?

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

        # removing publisher
        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

    def test_subscriber_withhold_update_disappear(self):
        """
        Test topic withholding functionality for a topic which doesnt exists anymore in the ros environment.
        Normal usecase
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        topicname = '/test/nonexistent3'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())
        # First update should not change state
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in self.subscriber_if_pool.subscribers_available:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                self.assertEqual(dt.added, [])  # nothing added (not exposed)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control
        # TODO : do we need a test with subscriber ?

        self.assertTrue(not t.timed_out)
        # topic should be in the list of args yet (not exposed)
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        # Here we are sure the internal state has topic_name registered
        # will be exposed right away
        dt = self.subscriber_if_pool.expose_subscribers([topicname])
        self.assertTrue(topicname in dt.added)  # detected and added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should STILL be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # topic backend should STILL be there
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

        dt = self.subscriber_if_pool.expose_subscribers([])
        self.assertTrue(topicname in dt.removed)  # removed
        self.assertEqual(dt.added, [])  # nothing added
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        nonexistent_pub.unregister()

        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        # Waiting a bit until the system state is back as expected (with connection cache this can be delayed)
        with Timeout(5) as t:
            while not t.timed_out and not topicname in self.subscriber_if_pool.subscribers_available:
                time.sleep(1)

    def test_subscriber_disappear_update_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """

        topicname = '/test/nonexistent4'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())
        # First update should not change state
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        dt = self.subscriber_if_pool.expose_subscribers([topicname])
        self.assertEqual(dt.added, [])  # nothing added (topic name doesnt exist yet)
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # topic backend has been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        def appear_disappear():
            # create the publisher and then try exposing the topic again, simulating
            # it coming online before expose call.
            nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                    subscribers, topic_types = self.get_system_state()
                    dt = self.subscriber_if_pool.update(subscribers, topic_types)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # detected
            # TODO : do we need a test with subscriber ?

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
            # topic backend has been created
            self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

            # up to here possible sequences should have been already tested by previous tests
            # Now comes our actual disappearance / withholding test
            nonexistent_pub.unregister()

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
            # the backend should STILL be there
            self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())
            # Note the Topic implementation should take care of possible errors in this case

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and topicname not in dt.removed:
                    subscribers, topic_types = self.get_system_state()
                    dt = self.subscriber_if_pool.update(subscribers, topic_types)
                    self.assertEqual(dt.added, [])  # nothing added
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(topicname in dt.removed)  # detected lost
            # every exposed topic should remain in the list of args ( in case regex match another topic )
            self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
            # make sure the topic backend should NOT be there any longer
            self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        appear_disappear()

        # test that coming back actually also works

        # Note : for the cache version we need update to run in between to make sure the cache callback for dropped
        # interface doesnt interfere with the new topic coming up.
        # TODO fix this problem (cache node / interface update not spinning fast enough to detect the change)
        # HOW ?
        # -> cache node (or proxy??) with possibility to mask subscribers + node to not trigger change ???
        # -> splitting subscribers and publishers will make it better to not confuse what goes up/down ???
        # -> wait for deletion confirmation before diff reporting deleted ? in update loop or higher, at test/user level ?
        # Currently there are tricks in place to determine if a diff comes from interface pub/sub creation/deletion
        #
        time.sleep(2)
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)

        # test that coming back actually also works
        appear_disappear()

        self.subscriber_if_pool.expose_subscribers([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertTrue(topicname not in dt.added)  # not detected
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

    def test_subscriber_update_disappear_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """

        topicname = '/test/nonexistent5'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())
        # First update should not change state
        subscribers, topic_types = self.get_system_state()
        dt = self.subscriber_if_pool.update(subscribers, topic_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        dt = self.subscriber_if_pool.expose_subscribers([topicname])
        self.assertEqual(dt.added, [])  # nothing added yet ( not existing )
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Subscriber(topicname, Empty, queue_size=1)

        with Timeout(5) as t:
            dt = DiffTuple([], [])
            while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                subscribers, topic_types = self.get_system_state()
                dt = self.subscriber_if_pool.update(subscribers, topic_types)
                self.assertEqual(dt.removed, [])  # nothing removed
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # added now because it just appeared
        self.assertEqual(dt.removed, [])  # nothing removed
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # topic backend has been created
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearrence / withholding test

        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )
        # TODO : test disappear ( how ? )

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers_args)
        # the backend should STILL be there
        self.assertTrue(topicname in self.subscriber_if_pool.subscribers.keys())
        # Note the Topic implementation should take care of possible errors in this case

        self.subscriber_if_pool.expose_subscribers([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers_args)
        # topic backend should NOT be there any longer
        self.assertTrue(topicname not in self.subscriber_if_pool.subscribers.keys())


#TODO : here we always test the full update => test the diff algorithm as well !

@nose.tools.istest
class TestRosInterface1NoCache(TestRosSubscriberIfPool):

    @nose.tools.nottest
    def get_system_state(self):

        publishers, subscribers, services = self._master.getSystemState()[2]
        topic_types = self._master.getTopicTypes()[2]

        return subscribers, topic_types

    def setUp(self):
        self.strsub = rospy.Subscriber('/test/string', String, queue_size=1)
        self.empsub = rospy.Subscriber('/test/empty', Empty, queue_size=1)

        self._master = rospy.get_master()
        self.subscriber_if_pool = RosSubscriberIfPool()

    def tearDown(self):
        self.subscriber_if_pool = None


# Testing with Connection Cache
@nose.tools.istest
class TestRosInterfaceCache(TestRosSubscriberIfPool):

    @nose.tools.nottest
    def get_system_state(self):
        # Note getting the system state via this interface, there is no need for callback
        publishers, subscribers, services = self.connection_cache.getSystemState()
        topic_types = self.connection_cache.getTopicTypes()

        return subscribers, topic_types

    def setUp(self):

        # first we setup our publishers and our node (used by rospy.resolve_name calls to remap topics)
        self.strsub = rospy.Subscriber('/test/string', String, queue_size=1)
        self.empsub = rospy.Subscriber('/test/empty', Empty, queue_size=1)

        self._master = rospy.get_master()
        self.subscriber_if_pool = RosSubscriberIfPool()

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

        self.connection_cache = connection_cache_proxy_create()

    def tearDown(self):
        self.subscriber_if_pool = None

        self.connection_cache_proc.stop()
        while self.connection_cache_proc.is_alive():
            time.sleep(0.2)  # waiting for cache node to die
        assert not self.connection_cache_proc.is_alive()
        time.sleep(1)  # TODO : investigate : we shouldnt need this


if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_subscriber_if_pool_no_cache', 'test_all', TestRosInterface1NoCache)

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_subscriber_if_pool_cache', 'test_all', TestRosInterfaceCache)
