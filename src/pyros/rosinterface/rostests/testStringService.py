#!/usr/bin/env python
from __future__ import absolute_import

import sys
import os
# to be able to run from source with rostest
devel_py_pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if os.path.exists(devel_py_pkg):
    sys.path.append(devel_py_pkg)

import logging

# nose get this from source ( specific nose import behavior for unit tests )
# rostest get this from devel workspace
import pyros
logging.info("pyros imported from {path}".format(path=pyros.__file__))
# Unit test import
from pyros.rosinterface import message_conversion as msgconv
from pyros.rosinterface import ServiceBack

# ROS imports should now work from ROS or from python (with or without ROS env setup - emulated if needed)
import rospy
import roslaunch
import rosservice
from std_msgs.msg import String, Empty
from pyros.srv import StringEchoService

#useful test tools
from pyros.rosinterface.tests import rostest_nose
import logging
import inspect
import unittest
import nose
from nose.tools import assert_false, assert_true, assert_equal


# test node process not setup by default (rostest dont need it here)
pub_process = None
echo_process = None


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match

        global pub_process, echo_process

        rospy.set_param('/echo_node/echo_service_name', 'test_service')
        echo_node = roslaunch.core.Node('pyros', 'string_echo_node.py', name='echo_node')

        echo_process = launch.launch(echo_node)

        # TODO : also use pub and sub nodes in more granular tests

        # set required parameters - needs to match the content of *.test files for rostest to match
        rospy.set_param('/stringServiceTest/echo_service_name', 'test_service')

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

        rospy.signal_shutdown('test complete')

        rostest_nose.rostest_nose_teardown_module()


class TestStringService(unittest.TestCase):
    """ Testing the TopicBack class with String message """
    # misc method
    def logPoint(self):
        currentTest = self.id().split('.')[-1]
        callingFunction = inspect.stack()[1][3]
        print 'in %s - %s()' % (currentTest, callingFunction)

    def service_wait_type(self, service_name, retries=5, retrysleep=1):
        resolved_service_name = rospy.resolve_name(service_name)
        service_type = rosservice.get_service_type(resolved_service_name)
        retry = 0
        while not service_type and retry < 5:
            print 'Service {service} not found. Retrying...'.format(service=resolved_service_name)
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            service_type = rosservice.get_service_type(resolved_service_name)
        if retry >= retries:
            self.fail("service {0} not found ! Failing.".format(resolved_service_name))
        return service_type

    def setUp(self):
        self.logPoint()
        # Here we hook to the test service in supported ways
        param_name = "/stringServiceTest/echo_service_name"
        self.echo_service_name = rospy.get_param(param_name, "")
        print 'Parameter {p} has value {v}'.format(p=param_name, v=self.echo_service_name)
        if self.echo_service_name == "":
            self.fail("{0} parameter not found".format(param_name))

        # No need of parameter for that, any str should work
        self.test_message = "testing"

        try:
            # actual fixture stuff
            # looking for the topic ( similar code than ros_interface.py )
            echo_service_type = self.service_wait_type(self.echo_service_name)

            # exposing the topic for testing here
            self.echo_service = ServiceBack(self.echo_service_name, echo_service_type)
        except KeyboardInterrupt:
            self.fail("Test Interrupted !")

    def tearDown(self):
        self.logPoint()
        pass

    def test_service(self):
        try:
            self.logPoint()

            print("calling : {msg} on topic {topic}".format(msg=self.test_message, topic=self.echo_service.name))
            resp = self.echo_service.call(self.echo_service.rostype_req(self.test_message))
            self.assertIn(resp.response, [self.test_message])

        except KeyboardInterrupt:
            self.fail("Test Interrupted !")


if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('pyros', 'testStringService', TestStringService, sys.argv)
