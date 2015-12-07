from __future__ import absolute_import

from . import ros_setup
try:
    import roslib
    import rospy
    from rospy.service import ServiceManager
    import rosservice, rostopic
    import actionlib_msgs.msg
except ImportError:
    ros_setup.ROS_emulate_setup()
    import roslib
    import rospy
    from rospy.service import ServiceManager
    import rosservice, rostopic
    import actionlib_msgs.msg


from importlib import import_module
from collections import OrderedDict

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
ServiceBack is the class handling conversion from Python API to ROS Service
"""
class ServiceBack:
    def __init__(self, service_name, service_type):
        self.name = service_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        service_type_module, service_type_name = tuple(service_type.split('/'))
        roslib.load_manifest(service_type_module)
        srv_module = import_module(service_type_module + '.srv')

        self.rostype_name = service_type
        self.rostype = getattr(srv_module, service_type_name)
        self.rostype_req = getattr(srv_module, service_type_name + 'Request')
        self.rostype_resp = getattr(srv_module, service_type_name + 'Response')

        self.srvtype = definitions.get_service_srv_dict(self)
        #rospy.logwarn('srvtype : %r', self.srvtype)

        self.proxy = rospy.ServiceProxy(self.name, self.rostype)

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """

        return OrderedDict({
            'name': self.name,
            'fullname': self.fullname,
            'srvtype': self.srvtype,
            'rostype_name': self.rostype_name,
            'rostype_req': self.rostype_req,
            'rostype_resp': self.rostype_resp,
        })

    def call(self, rosreq = None):
#       rosreq = self.rostype_req()
#       if use_ros:
#           rosreq.deserialize(req)
#       else:
#           msgconv.populate_instance(req, rosreq)

        fields = []
        if rosreq:
            for slot in rosreq.__slots__:
                fields.append(getattr(rosreq, slot))
            fields = tuple(fields)

        return self.proxy(*fields)
