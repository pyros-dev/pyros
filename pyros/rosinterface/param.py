from __future__ import absolute_import

import rospy
from collections import OrderedDict


class ParamTuple(object):
    def __init__(self, name, type):
        self.name = name
        self.type = type
# TODO: make that the pickled representation of ParamBack (check asdict())


class ParamBack(object):
    """
    ParamBack is the class handling conversion from REST API to ROS Param
    """
    def __init__(self, param_name):
        self.name = param_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

    def cleanup(self):
        pass

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
        })

    def setval(self, val):
        rospy.set_param(self.name, val)
        return

    def getval(self):
        res = rospy.get_param(self.name)
        return res
