#!/usr/bin/env python

""" A dummy ROS node """
import common
import rospy
import std_msgs

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "got %s", data.data)

if __name__ == '__main__':
    try:
        rospy.init_node('string_sub_node')
        rospy.loginfo('String Sub node started. [' + rospy.get_name() + ']')

        topic_name = rospy.get_param("~topic_name", "")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~topic_name'), topic_name)
        if topic_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~topic_name'))
            raise common.TestArgumentNotFound

        pub = rospy.Subscriber(topic_name, std_msgs.msg.String, callback, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
