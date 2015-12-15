#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse

def handle_msg(rq):
    print("Empty Node got request")
    # TODO: assert rq is Empty Request (needed or done by rospy already ?)
    return EmptyResponse()

def empty_server():
    rospy.init_node('empty_node')
    srv = rospy.Service('/test/empsrv', Empty, handle_msg)
    rospy.spin()

if __name__ == '__main__':
    empty_server()
