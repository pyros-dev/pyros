#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def handle_msg(rq):
    print("server got request")
    return Empty()

def empty_server():
    rospy.init_node('empty_server')
    srv = rospy.Service('/test/empsrv', Empty, handle_msg)
    rospy.spin()

if __name__ == '__main__':
    empty_server()
