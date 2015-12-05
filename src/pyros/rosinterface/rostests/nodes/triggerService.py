#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

def handle_msg(rq):
    print("server got request")
    return Trigger()

def trigger_server():
    rospy.init_node('trigger_server')
    srv = rospy.Service('/test/trgsrv', Trigger, handle_msg)
    rospy.spin()

if __name__ == '__main__':
    trigger_server()
