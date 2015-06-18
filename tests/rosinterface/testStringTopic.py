#!/usr/bin/env python

""" Testing the rosinterface.TopicBack function """

import unittest
import inspect
import sys

import rospy
import rostopic
import rostest
import std_msgs

import rosinterface

class TopicNotFound(Exception):
    pass

class TestStringTopic(unittest.TestCase):
    # misc method
    def logPoint(self):
        currentTest = self.id().split('.')[-1]
        callingFunction = inspect.stack()[1][3]
        print 'in %s - %s()' % (currentTest, callingFunction)

    def setUp(self):
        self.logPoint()
        """
        Non roslaunch fixture setup method
        :return:
        """
        #Here we hook to the test topic in supported ways
        param_name = "/stringTopicTest/topic_name"
        topic_name = rospy.get_param(param_name, "")
        print 'Parameter {p} has value {v}'.format(p=param_name, v=topic_name)
        if topic_name == "":
            self.fail("{0} parameter not found".format(param_name))

        try:
            # actual fixture stuff
            # looking for the topic ( similar code than ros_interface.py )
            resolved_topic_name = rospy.resolve_name(topic_name)
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            retry = 0
            while not topic_type and retry < 5:
                print 'Topic {topic} not found'.format(topic=resolved_topic_name)
                rospy.rostime.wallsleep(1)
                retry += 1
                topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if retry >= 5:
                raise TopicNotFound

            # exposing the topic for testing here
            self.test_topic_sub = rosinterface.TopicBack(topic_name, topic_type, allow_pub=False, allow_sub=True)
            self.test_topic_pub = rosinterface.TopicBack(topic_name, topic_type, allow_pub=True, allow_sub=False)
            self.test_topic = rosinterface.TopicBack(topic_name, topic_type, allow_pub=True, allow_sub=True)
        except KeyboardInterrupt:
            self.fail("Test Interrupted !")

    def tearDown(self):
        self.logPoint()
        """
        Non roslaunch fixture teardown method
        :return:
        """
        pass


    def test_topic(self):
        self.logPoint()
        pass



if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    rostest.rosrun('rostful-node', 'testStringTopic', TestStringTopic, sys.argv)

