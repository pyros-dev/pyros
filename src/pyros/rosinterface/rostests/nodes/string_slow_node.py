#!/usr/bin/env python
from __future__ import absolute_import

import sys
import os

"""
 A very simple echo ROS node, with delay.
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

def service_callback(data):
    # extract data
    print "==> sleeping {delay} seconds ".format(delay=30)
    rospy.rostime.wallsleep(30)
    print "==> finally replying to {d} ".format(d=data)
    return "response"
    # TODO : generic way to forward any msgtype safely

if __name__ == '__main__':
    try:
        rospy.init_node('string_slow_node')
        rospy.loginfo('String Slow node started. [' + rospy.get_name() + ']')

        slow_service_name = rospy.get_param("~slow_service_name", "slow_service")
        print 'Parameter {0!s} has value {1!s}'.format(rospy.resolve_name('~slow_service_name'), slow_service_name)
        if slow_service_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~slow_service_name'))
            raise common.TestArgumentNotFound

        srv = rospy.Service(slow_service_name, StringEchoService, service_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

# TO RUN :
# roscore &
# rosrun pyros string_echo_node.py