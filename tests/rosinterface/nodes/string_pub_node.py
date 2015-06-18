#!/usr/bin/env python

""" A dummy ROS node """
from __future__ import absolute_import

import rospy
import std_msgs
import common

if __name__ == '__main__':
    try:
        rospy.init_node('pub_node')
        rospy.loginfo('pub node started. [' + rospy.get_name() + ']')

        topic_name = rospy.get_param("~topic_name", "")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~topic_name'), topic_name)
        if topic_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~topic_name'))
            raise common.TestArgumentNotFound

        pub = rospy.Publisher(topic_name, std_msgs.msg.String, queue_size=10)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            test_str = "test %s" % rospy.get_time()
            rospy.loginfo(test_str)
            pub.publish(test_str)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
