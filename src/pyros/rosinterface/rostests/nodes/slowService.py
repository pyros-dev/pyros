#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse


def handle_msg(rq):
    print("Slow Node got request")
    rospy.rostime.wallsleep(30)
    return EmptyResponse()


def empty_server():
    rospy.init_node('slow_node')
    srv = rospy.Service('/test/slowsrv', Empty, handle_msg)
    rospy.spin()

if __name__ == '__main__':
    empty_server()
