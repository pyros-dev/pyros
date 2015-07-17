from __future__ import absolute_import

import roslib
import rospy
from rospy.service import ServiceManager
import rosservice, rostopic
import actionlib_msgs.msg

from importlib import import_module
from collections import deque

import json
import sys
import re
from StringIO import StringIO

from . import message_conversion as msgconv
from . import deffile, definitions

from .util import ROS_MSG_MIMETYPE, request_wants_ros, get_query_bool

import os
import urlparse

"""
TopicBack is the class handling conversion from REST API to ROS Topic
"""
class TopicBack:
    def __init__(self, topic_name, topic_type, allow_pub=True, allow_sub=True, queue_size=1):
        self.name = topic_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        topic_type_module, topic_type_name = tuple(topic_type.split('/'))
        roslib.load_manifest(topic_type_module)
        msg_module = import_module(topic_type_module + '.msg')

        self.rostype_name = topic_type
        self.rostype = getattr(msg_module, topic_type_name)

        self.allow_pub = allow_pub
        self.allow_sub = allow_sub

        self.msgtype = definitions.get_topic_msg_dict(self)
        self.msg = deque([], queue_size)

        self.pub = None
        if self.allow_pub:
            self.pub = rospy.Publisher(self.name, self.rostype, queue_size=1)

        self.sub = None
        if self.allow_sub:
            self.sub = rospy.Subscriber(self.name, self.rostype, self.topic_callback)

        self.empty_cb = None

    def publish(self, msg):
        self.pub.publish(msg)
        return

    def get(self, num=0, consume=False):
        if not self.msg:
            return None

        res = None
        #TODO : implement returning multiple messages
        if consume:
            res = self.msg.popleft()
            if 0 == len(self.msg) and self.empty_cb:
                self.empty_cb()
                #TODO : CHECK that we can survive here even if we get dropped from the topic list
        else:
            res = self.msg[0]

        return res

    #returns the number of unread message
    def unread(self):
        return len(self.msg)

    def set_empty_callback(self, cb):
        self.empty_cb = cb

    def topic_callback(self, msg):
        self.msg.appendleft(msg)
