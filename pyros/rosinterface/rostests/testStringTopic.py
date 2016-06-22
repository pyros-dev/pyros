#!/usr/bin/env python
from __future__ import absolute_import

import six

import os
import sys

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

# Unit test import ( will emulate ROS setup if needed )
import time

from pyros.rosinterface import TopicBack


# ROS imports should now work from ROS or from python (without ROS env setup)
import rospy
import roslaunch
import rostopic
from std_msgs.msg import String, Empty

import pyros

from pyros_setup import rostest_nose
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

        rospy.set_param('/echo_node/topic_name', 'test_topic')
        rospy.set_param('/echo_node/echo_topic_name', 'echo_test_topic')
        echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='echo_node')
        try :
            echo_process = launch.launch(echo_node)
        except roslaunch.RLException as rlexc:
            logging.error("pyros_test is needed to run this test. Please verify that it is installed in your ROS environment")
            raise

        # TODO : also use pub and sub nodes in more granular tests

        # set required parameters - needs to match the content of *.test files for rostest to match
        rospy.set_param('/stringTopicTest/pub_topic_name', 'test_topic')
        rospy.set_param('/stringTopicTest/echo_topic_name', 'echo_test_topic')

        # we still need a node to interact with topics
        rospy.init_node('TestStringTopic', anonymous=True, disable_signals=True)
        # CAREFUL : this should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        # finishing all process are finished
        if pub_process is not None and pub_process.is_alive():
            pub_process.stop()
        if echo_process is not None and echo_process.is_alive():
            echo_process.stop()

        rostest_nose.rostest_nose_teardown_module()


# TODO : review this test (and ROS topic implementation) :
# The required functionality is to get the latest message on the topic
# But ideally we should never lose message on topic,
#   or behavior might be very different if message changes while one publisher spams it ( like here )
#  -> means we need an explicit way of consuming message
#  -> means we need paging to be able to access multiple messages
#  -> means we need memory and max reached behavior...
#  -> means we probably want to filter what message to keep or not
#  -> means we probably need an option to RLE compress messages in very fast topic rates ( "blahblah" arrived 5 times )
#  -> means we probably need a way to keep but greatly compress old messages ( logrotate style )

class TestStringTopic(unittest.TestCase):
    """ Testing the TopicBack class with String message """
    # misc method
    def logPoint(self):
        currentTest = self.id().split('.')[-1]
        callingFunction = inspect.stack()[1][3]
        print('in {0!s} - {1!s}()'.format(currentTest, callingFunction))

    def msg_extract(self, topic, retries=5, retrysleep=1):
        msg = topic.get()
        retry = 0
        while not msg and retry < retries:
            print('Message not extracted. Retrying...')
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            msg = topic.get()
        if retry >= retries:
            self.fail('Message not extracted. Failing.')
        else:
            return msg

    def msg_wait(self, expected_msg, topic, retries=5, retrysleep=1):
        msg = self.msg_extract(topic)
        retry = 0
        # set XOR is a quick way to diff dict : if empty we stop (dicts are equals)
        while retry < retries and (set(six.iteritems(msg)) ^ set(six.iteritems(expected_msg))):
            print('Discarding unexpected message {0}. Retrying...'.format(msg))
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            msg = self.msg_extract(topic)
            print('Checking msg \"{0}\" == s \"{1}\"'.format(msg, expected_msg))

        if retry >= retries:
            self.fail('No Expected message {0} arrived. Failing.'.format(expected_msg))
        else:
            return msg

    def topic_wait_type(self, topic_name, retries=5, retrysleep=1):
        resolved_topic_name = rospy.resolve_name(topic_name)
        topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
        retry = 0
        while not topic_type and retry < 5:
            print('Topic {topic} not found. Retrying...'.format(topic=resolved_topic_name))
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
        if retry >= retries:
            self.fail("Topic {0} not found ! Failing.".format(resolved_topic_name))
        return topic_type

    def setUp(self):
        self.logPoint()
        """
        Non roslaunch fixture setup method
        :return:
        """
        # Here we hook to the test topic in supported ways
        param_name = "/stringTopicTest/pub_topic_name"
        self.pub_topic_name = rospy.get_param(param_name, "")
        print('Parameter {p} has value {v}'.format(p=param_name, v=self.pub_topic_name))
        if self.pub_topic_name == "":
            self.fail("{0} parameter not found".format(param_name))

        param_name = "/stringTopicTest/echo_topic_name"
        self.echo_topic_name = rospy.get_param(param_name, "")
        print('Parameter {p} has value {v}'.format(p=param_name, v=self.echo_topic_name))
        if self.echo_topic_name == "":
            self.fail("{0} parameter not found".format(param_name))

        # No need of parameter for that, any str should work
        self.test_message = "testing"

    def tearDown(self):
        self.logPoint()
        pass

    def test_topic_echo(self):
        try:
            self.logPoint()

            # looking for the topic ( similar code than ros_interface.py )
            pub_topic_type = self.topic_wait_type(self.pub_topic_name)
            echo_topic_type = self.topic_wait_type(self.echo_topic_name)

            # exposing the topic for testing here
            self.pub_topic = TopicBack(self.pub_topic_name, pub_topic_type)
            self.echo_topic = TopicBack(self.echo_topic_name, echo_topic_type)

            # Making sure the topic interface is ready to be used just after creation
            subs_connected = self.pub_topic.pub.get_num_connections() > 0  # no local subs
            assert_true(subs_connected)
            pubs_connected = self.echo_topic.sub.get_num_connections() > 0  # no local pub
            assert_true(pubs_connected)

            # Topics are up. Use them.
            print("sending : {msg} on topic {topic}".format(msg=self.test_message, topic=self.pub_topic.name))
            assert_true(self.pub_topic.publish({'data': self.test_message}))

            print("waiting for : {msg} on topic {topic}".format(msg={'data': self.test_message}, topic=self.echo_topic.name))
            msg = self.msg_wait({'data': self.test_message}, self.echo_topic)
            # We assert there is no difference
            assert_true(len(set(six.iteritems(msg)) ^ set(six.iteritems({'data': self.test_message}))) == 0)

        except KeyboardInterrupt:
            self.fail("Test Interrupted !")


if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('pyros', 'testStringTopic', TestStringTopic, sys.argv)