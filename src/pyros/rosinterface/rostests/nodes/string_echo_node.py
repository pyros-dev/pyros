#!/usr/bin/env python
from __future__ import absolute_import

import sys
import os

"""
 A very simple echo ROS node.
 - echo from topic to echo_topic
 - echo service
"""
import functools

import common
import rospy
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs

# TODO : get rid of this somehow ( dynamic generation or integration of more basic services in ROS )
from pyros.srv import StringEchoService


# a callback that just echoes the message ( if it doesnt come from me )
def topic_callback(data, data_type, pub):
    # extract data
    print "==> echoing {d} ".format(d=data)
    pub.publish(data=data.data)  # data member is needed because it s the only std_msgs.String.__slots__
    # BAD ROS API is not symmetrical
    # TODO : generic way to forward any msgtype safely
    pass


def service_callback(data):
    # extract data
    print "==> echoing {d} ".format(d=data)
    return data.request  # why specific field here as well ? "request" / response but no "data" ?
    # TODO : generic way to forward any msgtype safely

if __name__ == '__main__':
    try:
        rospy.init_node('string_echo_node')
        rospy.loginfo('String Echo node started. [' + rospy.get_name() + ']')

        topic_name = rospy.get_param("~topic_name", "topic")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~topic_name'), topic_name)
        if topic_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~topic_name'))
            raise common.TestArgumentNotFound

        echo_topic_name = rospy.get_param("~echo_topic_name", "echo_topic")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~echo_topic_name'), echo_topic_name)
        if echo_topic_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~echo_topic_name'))
            raise common.TestArgumentNotFound

        echo_service_name = rospy.get_param("~echo_service_name", "echo_service")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~echo_service_name'), echo_service_name)
        if echo_service_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~echo_service_name'))
            raise common.TestArgumentNotFound

        # TODO parameter topic type to reuse this for *any* msg type

        pub = rospy.Publisher(echo_topic_name, std_msgs.String, queue_size=1)

        # building callback
        echo = functools.partial(topic_callback, data_type=std_msgs.String, pub=pub)
        sub = rospy.Subscriber(topic_name, std_msgs.String, echo)

        srv = rospy.Service(echo_service_name, StringEchoService, service_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

# TO RUN :
# roscore &
# rosrun pyros string_echo_node.py