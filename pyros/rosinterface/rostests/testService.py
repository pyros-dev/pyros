#!/usr/bin/env python
from __future__ import absolute_import

import six
import sys
import logging
import os
import pickle

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

# Unit test import
from pyros.rosinterface import ServiceBack


# ROS imports should now work from ROS or from python (with or without ROS env setup - emulated if needed)
import rospy
import roslaunch
import rosservice

#useful test tools
from pyros_setup import rostest_nose
import inspect
import unittest
import nose
from nose.tools import assert_false, assert_true, assert_equal


# test node process not setup by default (rostest dont need it here)
pub_process = None
echo_process = None
slow_process = None


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match

        global pub_process, echo_process, slow_process

        rospy.set_param('/echo_node/echo_service_name', 'test_service')
        echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='echo_node')
        try:
            echo_process = launch.launch(echo_node)
        except roslaunch.RLException as rlexc:
            logging.error("pyros_test is needed to run this test. Please verify that it is installed in your ROS environment")
            raise
        rospy.set_param('/slow_node/slow_service_name', 'test_timeout_service')
        slow_node = roslaunch.core.Node('pyros_test', 'string_slow_node.py', name='slow_node')
        try:
            slow_process = launch.launch(slow_node)
        except roslaunch.RLException as rlexc:
            logging.error("pyros_test is needed to run this test. Please verify that it is installed in your ROS environment")
            raise
        # set required parameters - needs to match the content of *.test files for rostest to match
        rospy.set_param('/stringServiceTest/echo_service_name', 'test_service')
        rospy.set_param('/stringServiceTest/slow_service_name', 'test_timeout_service')

        # we still need a node to interact with topics
        rospy.init_node('TestStringService', anonymous=True, disable_signals=True)
        # CAREFUL : this should be done only once per PROCESS
        # Here we enforce TEST MODULE 1<->1 PROCESS. ROStest style


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        # finishing all process are finished
        if pub_process is not None:
            pub_process.stop()
        if echo_process is not None:
            echo_process.stop()
        if slow_process is not None:
            slow_process.stop()

        rostest_nose.rostest_nose_teardown_module()


class TestService(unittest.TestCase):
    """ Testing the TopicBack class with String message """
    # misc method
    def logPoint(self):
        currentTest = self.id().split('.')[-1]
        callingFunction = inspect.stack()[1][3]
        print('in {0!s} - {1!s}()'.format(currentTest, callingFunction))

    def service_wait_type(self, service_name, retries=5, retrysleep=1):
        resolved_service_name = rospy.resolve_name(service_name)
        service_type = rosservice.get_service_type(resolved_service_name)
        retry = 0
        while not service_type and retry < 5:
            print('Service {service} not found. Retrying...'.format(service=resolved_service_name))
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            service_type = rosservice.get_service_type(resolved_service_name)
        if retry >= retries:
            self.fail("service {0} not found ! Failing.".format(resolved_service_name))
        return service_type

    def setUp(self):
        self.logPoint()
        # Here we hook to the test service in supported ways
        self.echo_service_name = rospy.get_param("/stringServiceTest/echo_service_name", "")
        self.slow_service_name = rospy.get_param("/stringServiceTest/slow_service_name", "")
        # No need of parameter for that, any str should work
        self.test_message = "testing"

        try:
            # actual fixture stuff
            # looking for the service ( similar code than ros_interface.py )
            echo_service_type = self.service_wait_type(self.echo_service_name)

            # exposing the service for testing here
            self.echo_service = ServiceBack(self.echo_service_name, echo_service_type)
            # looking for the service ( similar code than ros_interface.py )
            slow_service_type = self.service_wait_type(self.slow_service_name)

            # exposing the service for testing here
            self.slow_service = ServiceBack(self.slow_service_name, slow_service_type)
        except KeyboardInterrupt:
            self.fail("Test Interrupted !")

    def tearDown(self):
        self.logPoint()
        pass

    def test_service(self):
        try:
            self.logPoint()

            print("calling : {msg} on service {service}".format(msg=self.test_message, service=self.echo_service.name))
            resp = self.echo_service.call({'request': self.test_message})
            # Assert there is no difference
            assert_true(len(set(six.iteritems(resp)) ^ set(six.iteritems({'response': self.test_message}))) == 0)

        except KeyboardInterrupt:
            self.fail("Test Interrupted !")

    #TODO
    # def test_slow_service_timeout(self):
    #
    #     try:
    #         self.logPoint()
    #
    #         print("calling : {msg} on service {service}".format(msg=self.test_message, service=self.slow_service.name))
    #         with self.assertRaises(TypeError):
    #         resp = self.slow_service.call(self.slow_service.rostype_req(self.test_message))
    #         self.assertIn(resp.response, [self.test_message])
    #
    #     except KeyboardInterrupt:
    #         self.fail("Test Interrupted !")


if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('pyros', 'testService', TestService, sys.argv)
