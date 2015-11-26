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
ParamBack is the class handling conversion from REST API to ROS Param
"""
class ParamBack:
    def __init__(self, param_name):
        self.name = param_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

    def set(self, val):
        rospy.set_param(self.name, val)
        return

    def get(self):
        res = rospy.get_param(self.name)
        return res
