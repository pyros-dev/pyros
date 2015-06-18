#!/usr/bin/env python

""" An echo ROS node """
import functools
from StringIO import StringIO

import common
import rospy
import std_msgs

# a callback that just echoes the message ( if it doesnt come from me )
def callback(prefix, pub_topic, data):

    # extract data
    recvd_data = StringIO()
    if data is not None:
        data.serialize(recvd_data)
    recvd_data = recvd_data.getvalue()

    #print "received {d} ".format(d=recvd_data)
    if prefix not in recvd_data:  # based on std_msgs.String msg format
        pub_topic.publish(prefix + recvd_data)
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('string_echo_node')
        rospy.loginfo('String Echo node started. [' + rospy.get_name() + ']')

        topic_name = rospy.get_param("~topic_name", "")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~topic_name'), topic_name)
        if topic_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~topic_name'))
            raise common.TestArgumentNotFound
        echo_prefix = rospy.get_param("~echo_prefix", "")
        print 'Parameter %s has value %s' % (rospy.resolve_name('~echo_prefix'), echo_prefix)
        if echo_prefix == "":
            print "{0} parameter not found".format(rospy.resolve_name('~echo_prefix'))
            raise common.TestArgumentNotFound

        pub = rospy.Publisher(topic_name, std_msgs.msg.String, queue_size=1)
        # building callback
        echo = functools.partial(callback, echo_prefix, pub)
        sub = rospy.Subscriber(topic_name, std_msgs.msg.String, echo, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
