#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse


def handle_msg(rq):
    print("Trigger Node got request")
    return TriggerResponse(success=True, message="trigger received")


def trigger_server():
    rospy.init_node('trigger_node')
    srv = rospy.Service('/test/trgsrv', Trigger, handle_msg)
    rospy.spin()


if __name__ == '__main__':
    trigger_server()
