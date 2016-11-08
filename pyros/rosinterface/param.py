from __future__ import absolute_import
from __future__ import print_function

from collections import OrderedDict

from .api import rospy_safe as rospy
from ..baseinterface import TransientIf


class ParamTuple(object):
    def __init__(self, name, type):
        self.name = name
        self.type = type
# TODO: make that the pickled representation of ParamBack (check asdict())


class ParamBack(TransientIf):
    """
    ParamBack is the class handling conversion from REST API to ROS Param
    """
    def __init__(self, param_name, param_type):
        param_name = rospy.resolve_name(param_name)

        # defining a type on param to unify API for topic, services and param.
        super(ParamBack, self).__init__(param_name, param_type)

        # TODO : should we here manage how to get param values with rospy (same as for service and topic ?)

    def cleanup(self):
        super(ParamBack, self).cleanup()

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """

        return OrderedDict({
            'name': self.name,
            'fullname': self.name,  # for BWcompat
            'prmtype': self.type,
        })

    def setval(self, val):
        # TODO : think about stricter type checking...
        rospy.set_param(self.name, val)
        return

    def getval(self):
        res = rospy.get_param(self.name)
        return res
