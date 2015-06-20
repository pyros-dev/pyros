#!/usr/bin/env python

""" Testing the rosinterface.TopicBack function """

import unittest
import inspect
import sys

import rospy
import rostopic
import rostest

import rosinterface

class TestStringTopic(unittest.TestCase):
    # misc method
    def logPoint(self):
        currentTest = self.id().split('.')[-1]
        callingFunction = inspect.stack()[1][3]
        print 'in %s - %s()' % (currentTest, callingFunction)

    def msg_extract(self, retries=5, retrysleep=1):
        msg = self.topic.get()
        retry = 0
        while not msg and retry < retries:
            print 'Message not extracted. Retrying...'
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            msg = self.topic.get()
        if retry >= retries:
            self.fail('Message not extracted. Failing.')
        else:
            return msg

    def msg_wait(self, strings, retries=5, retrysleep=1):
        msg = self.msg_extract()
        retry = 0
        while retry < retries and not msg in strings:
            print 'Discarding unexpected message {0}. Retrying...'.format(msg)
            rospy.rostime.wallsleep(retrysleep)
            retry += 1
            msg = self.msg_extract()
            for s in strings:
                if msg == s:
                    print('msg \"{0}\" == s \"{1}\"'.format(msg, s))
                else:
                    print('msg \"{0}\" != s \"{1}\"'.format(msg, s))
        if retry >= retries:
            self.fail('No Expected message {0} arrived. Failing.'.format(strings))
        else:
            return msg

    def setUp(self):
        self.logPoint()
        """
        Non roslaunch fixture setup method
        :return:
        """
        #Here we hook to the test topic in supported ways
        param_name = "/stringTopicTest/topic_name"
        self.topic_name = rospy.get_param(param_name, "")
        print 'Parameter {p} has value {v}'.format(p=param_name, v=self.topic_name)
        if self.topic_name == "":
            self.fail("{0} parameter not found".format(param_name))

        self.echo_prefix = rospy.get_param("/stringTopicTest/echo_prefix", "")
        print 'Parameter %s has value %s' % (rospy.resolve_name("/stringTopicTest/echo_prefix"), self.echo_prefix)
        if self.echo_prefix == "":
            self.fail("{0} parameter not found".format(rospy.resolve_name("/stringTopicTest/echo_prefix")))

        self.test_message = rospy.get_param("/stringTopicTest/test_message", "")
        print 'Parameter %s has value %s' % (rospy.resolve_name('/stringTopicTest/test_message'), self.test_message)
        if self.test_message == "":
            self.fail("{0} parameter not found".format(rospy.resolve_name('/stringTopicTest/test_message')))

        # we still need a node to interact with topics
        rospy.init_node('TestStringTopic', anonymous=True, disable_signals=True)

        try:
            # actual fixture stuff
            # looking for the topic ( similar code than ros_interface.py )
            resolved_topic_name = rospy.resolve_name(self.topic_name)
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            retry = 0
            while not topic_type and retry < 5:
                print 'Topic {topic} not found. Retrying...'.format(topic=resolved_topic_name)
                rospy.rostime.wallsleep(1)
                retry += 1
                topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if retry >= 5:
                self.fail("Topic {0} not found ! Failing.".format(resolved_topic_name))

            # exposing the topic for testing here
            self.topic_sub = rosinterface.TopicBack(self.topic_name, topic_type, allow_pub=False, allow_sub=True)
            self.topic_pub = rosinterface.TopicBack(self.topic_name, topic_type, allow_pub=True, allow_sub=False)
            self.topic = rosinterface.TopicBack(self.topic_name, topic_type, allow_pub=True, allow_sub=True)
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
        try:
            self.logPoint()
            # Test message on topic
            msg = self.msg_wait([self.test_message, self.echo_prefix + self.test_message])
            self.assertIn(msg, [self.test_message, self.echo_prefix + self.test_message])

            # send message
            self.topic.publish(self.test_message + self.test_message)

            # wait for echo
            msg = self.msg_wait([self.echo_prefix + self.test_message + self.test_message])
            self.assertIn(msg, [self.echo_prefix + self.test_message + self.test_message])

        except KeyboardInterrupt:
            self.fail("Test Interrupted !")

    def test_topic_sub(self):
        try:
            self.logPoint()
            # Test message on topic
            msg = self.msg_wait([self.test_message, self.echo_prefix + self.test_message])
            self.assertIn(msg, [self.test_message, self.echo_prefix + self.test_message])

            # with self.assertRaises():
            # send message
            self.topic.publish(self.test_message + self.test_message)

            # wait for echo
            msg = self.msg_wait([self.echo_prefix + self.test_message + self.test_message])
            self.assertIn(msg, [self.echo_prefix + self.test_message + self.test_message])
        except KeyboardInterrupt:
            self.fail("Test Interrupted !")

    def test_topic_pub(self):
        try:
            self.logPoint()
            # with self.assertRaises():
            # Test message on topic
            msg = self.msg_wait([self.test_message, self.echo_prefix + self.test_message])
            self.assertIn(msg, [self.test_message, self.echo_prefix + self.test_message])

            # send message
            self.topic.publish(self.test_message + self.test_message)

            # wait for echo
            msg = self.msg_wait([self.echo_prefix + self.test_message + self.test_message])
            self.assertIn(msg, [self.echo_prefix + self.test_message + self.test_message])
        except KeyboardInterrupt:
            self.fail("Test Interrupted !")


if __name__ == '__main__':
    print("ARGV : %r", sys.argv)
    rostest.rosrun('rostful-node', 'testStringTopic', TestStringTopic, sys.argv)

