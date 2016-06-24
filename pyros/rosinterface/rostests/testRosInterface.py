#!/usr/bin/env python
from __future__ import absolute_import

import os
import sys
import pickle

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
import time

current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec


# Unit test import (  will emulate ROS setup if needed )
import nose

from pyros.baseinterface import DiffTuple
from pyros.rosinterface import RosInterface, TopicBack


import rospy
import roslaunch
import rosnode
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

from pyros.rosinterface.rostests import Timeout

# useful test tools
from pyros_setup import rostest_nose
import unittest



# This should have the same effect as the <name>.test file for rostest. Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request

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
        # finishing all process are finished
        if TestRosInterface.empty_srv_process is not None:
            TestRosInterface.empty_srv_process.stop()
        if TestRosInterface.trigger_srv_process is not None:
            TestRosInterface.trigger_srv_process.stop()

# EXPOSE TOPICS + UPDATE Interface
    def test_topic_appear_expose_update(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """
        topicname = '/test/string'
        self.interface.expose_topics([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.interface.topics.keys())
        dt = self.interface.update()
        self.assertTrue(topicname in dt.added)  # has been detected

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.topics_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.topics.keys())

        # cleaning up
        self.interface.expose_topics([])
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # topic backend has not been created since the update didn't run yet
        self.assertTrue(topicname not in self.interface.topics.keys())

    def test_topic_appear_update_expose(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """

        topicname = '/test/nonexistent1'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = TopicBack._create_pub(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and nonexistent_pub.resolved_name not in self.interface.topics_available:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                self.assertEqual(dt.removed, [])  # nothing removed

        self.assertTrue(not t.timed_out)
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        self.interface.expose_topics([topicname])
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.topics_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.topics.keys())

        # removing publisher
        TopicBack._remove_pub(nonexistent_pub)  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

        # and update should be enough to cleanup
        with Timeout(5) as t:
            while not t.timed_out and not topicname in dt.removed:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.removed)
        self.assertTrue(topicname not in self.interface.topics_available)

        # every exposed topic should still be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # the backend should not be there any longer
        self.assertTrue(topicname not in self.interface.topics.keys())

    def test_topic_expose_appear_update(self):
        """
        Test basic topic adding functionality for a topic which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        topicname = '/test/nonexistent2'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        self.interface.expose_topics([topicname])
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())
        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # make sure the topic is STILL in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # make sure the topic backend has STILL not been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        # create the publisher and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in dt.added:
                dt = self.interface.update()
                self.assertEqual(dt.removed, [])  # nothing removed

        self.assertTrue(not t.timed_out)
        self.assertTrue(topicname in dt.added)  # detected
        # TODO : do we need a test with subscriber ?

        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.topics_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.topics.keys())

        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )

    def test_topic_withhold_update_disappear(self):
        """
        Test topic withholding functionality for a topic which doesnt exists anymore in the ros environment.
        Normal usecase
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        topicname = '/test/nonexistent3'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = TopicBack._create_pub(topicname, Empty, queue_size=1)
        with Timeout(5) as t:
            while not t.timed_out and topicname not in self.interface.topics_available:
                dt = self.interface.update()
                self.assertEqual(dt.added, [])  # nothing added (not exposed)
                self.assertEqual(dt.removed, [])  # nothing removed
        # TODO : do we need a test with subscriber ?

        self.assertTrue(not t.timed_out)
        # topic should be in the list of args yet (not exposed)
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        # Here we are sure the internal state has topic_name registered
        # will be exposed right away
        dt = self.interface.expose_topics([topicname])
        self.assertTrue(topicname in dt.added)  # detected and added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every exposed topic should remain in the list of args ( in case regex match another topic )
        self.assertTrue(topicname in self.interface.topics_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.topics.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should STILL be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # topic backend should STILL be there
        self.assertTrue(topicname in self.interface.topics.keys())

        dt = self.interface.expose_topics([])
        self.assertTrue(topicname in dt.removed)  # removed
        self.assertEqual(dt.added, [])  # nothing added
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.topics.keys())

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.topics.keys())

        TopicBack._remove_pub(nonexistent_pub)

        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # topic backend should be GONE
        self.assertTrue(topicname not in self.interface.topics.keys())

    def test_topic_disappear_update_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in the ros environment.
        Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """

        topicname = '/test/nonexistent4'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        dt = self.interface.expose_topics([topicname])
        self.assertEqual(dt.added, [])  # nothing added (topic name doesnt exist yet)
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # topic backend has been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        def appear_disappear():
            # create the publisher and then try exposing the topic again, simulating
            # it coming online before expose call.
            nonexistent_pub = TopicBack._create_pub(topicname, Empty, queue_size=1)

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                    dt = self.interface.update()
                    self.assertEqual(dt.removed, [])  # nothing removed

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # detected
            # TODO : do we need a test with subscriber ?

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.interface.topics_args)
            # topic backend has been created
            self.assertTrue(topicname in self.interface.topics.keys())

            # up to here possible sequences should have been already tested by previous tests
            # Now comes our actual disappearance / withholding test
            TopicBack._remove_pub(nonexistent_pub)

            # every added topic should be in the list of args
            self.assertTrue(topicname in self.interface.topics_args)
            # the backend should STILL be there
            self.assertTrue(topicname in self.interface.topics.keys())
            # Note the Topic implementation should take care of possible errors in this case

            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and topicname not in dt.removed:
                    dt = self.interface.update()
                    self.assertEqual(dt.added, [])  # nothing added

            self.assertTrue(not t.timed_out)
            self.assertTrue(topicname in dt.removed)  # detected lost
            # every exposed topic should remain in the list of args ( in case regex match another topic )
            self.assertTrue(topicname in self.interface.topics_args)
            # make sure the topic backend should NOT be there any longer
            self.assertTrue(topicname not in self.interface.topics.keys())

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

        self.interface.expose_topics([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        dt = self.interface.update()
        self.assertTrue(topicname not in dt.added)  # not detected
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # make sure the topic backend has been created
        self.assertTrue(topicname not in self.interface.topics.keys())

    def test_topic_update_disappear_withhold(self):
        """
        Test topic exposing functionality for a topic which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """

        topicname = '/test/nonexistent5'
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())
        # First update should not change state
        dt = self.interface.update()
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        dt = self.interface.expose_topics([topicname])
        self.assertEqual(dt.added, [])  # nothing added yet ( not existing )
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # topic backend has not been created
        self.assertTrue(topicname not in self.interface.topics.keys())

        # create the publisher and then try exposing the topic again, simulating
        # it coming online before expose call.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)

        with Timeout(5) as t:
            dt = DiffTuple([], [])
            while not t.timed_out and nonexistent_pub.resolved_name not in dt.added:
                dt = self.interface.update()
                self.assertEqual(dt.removed, [])  # nothing removed

        self.assertTrue(not t.timed_out)
        self.assertTrue(nonexistent_pub.resolved_name in dt.added)  # added now because it just appeared
        self.assertEqual(dt.removed, [])  # nothing removed
        # TODO : do we need a test with subscriber ?

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # topic backend has been created
        self.assertTrue(topicname in self.interface.topics.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearrence / withholding test

        nonexistent_pub.unregister()  # https://github.com/ros/ros_comm/issues/111 ( topic is still registered on master... )
        # TODO : test disappear ( how ? )

        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # the backend should STILL be there
        self.assertTrue(topicname in self.interface.topics.keys())
        # Note the Topic implementation should take care of possible errors in this case

        self.interface.expose_topics([])
        # every withhold topic should NOT be in the list of args
        self.assertTrue(topicname not in self.interface.topics_args)
        # topic backend should NOT be there any longer
        self.assertTrue(topicname not in self.interface.topics.keys())



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


@nose.tools.istest
class TestRosInterfaceNoCache(TestRosInterface):
    def setUp(self):
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self.interface = RosInterface('test_rosinterface', enable_cache=False)

        # CAREFUL : this is doing a rospy.init_node, and it should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.

    def tearDown(self):
        self.interface = None

if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_ros_interface', 'test_all', TestRosInterface)

