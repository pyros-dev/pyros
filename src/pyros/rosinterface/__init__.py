# -*- coding: utf-8 -*-
#This python package is handling all ROS related communication for rostful-node.
from __future__ import absolute_import

import os
import types
import pyros_setup

"""
Hopefully this should endup in rosinterface.__doc__
"""

# class to allow recursive delayed conditional import.
# this way it can work with or without preset environment
class _PyrosSetup(types.ModuleType):


    def __init__(self, topic_back, service_back, param_back, ros_interface, pyros_ros):
        super(_PyrosSetup, self).__init__('RosInterface', 'Dynamically generated module to interface with ROS')
        # members simulating a usual imported module
        self.TopicBack = topic_back
        self.ServiceBack = service_back
        self.ParamBack = param_back
        self.RosInterface = ros_interface
        self.PyrosROS = pyros_ros
        # subpackages
        self.rostests = __import__("pyros.rosinterface.rostests", fromlist=["pyros.rosinterface"])
        self.tests = __import__("pyros.rosinterface.tests", fromlist=["pyros.rosinterface"])
        self.roconinterface = __import__("pyros.rosinterface.roconinterface", fromlist=["pyros.rosinterface"])
        # submodules for individual access
        self.message_conversion = __import__("pyros.rosinterface.message_conversion", fromlist=["pyros.rosinterface"])
        # TODO : we shouldnt need it...

    # encapsulating local imports to delay them until ROS setup is done
    @staticmethod
    def delayed_import(distro=None, *workspaces):
        distro = distro or 'indigo'
        try:
            import rospy  # early except to prevent unintentional workaround in all modules here
            from .topic import TopicBack
            from .service import ServiceBack
            from .param import ParamBack
            from .ros_interface import RosInterface
            from .pyros_ros import PyrosROS
        except ImportError:
            pyros_setup.delayed_import(distro, *workspaces)
            import rospy
            from .topic import TopicBack
            from .service import ServiceBack
            from .param import ParamBack
            from .ros_interface import RosInterface
            from .pyros_ros import PyrosROS

        # we return a relay of imported names, accessible the same way a direct import would be.
        return _PyrosSetup(TopicBack, ServiceBack, ParamBack, RosInterface, PyrosROS)

    @staticmethod
    def delayed_import_auto(distro=None, base_path=None):
        import os

        distro = distro or 'indigo'
        # default basepath working if pyros-setup is directly cloned in your workspace
        # This file think it is in devel/lib/python2.7/dist-packages/pyros_setup for some reason...
        # NOTE : maybe the default here is a bad idea, making it simpler than it should be for the user...
        base_path = base_path or os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')

        try:
            import rospy  # early except to prevent unintentional workaround in all modules here
            from .topic import TopicBack
            from .service import ServiceBack
            from .param import ParamBack
            from .ros_interface import RosInterface
            from .pyros_ros import PyrosROS
        except ImportError:
            pyros_setup.delayed_import_auto(distro, base_path)
            from .topic import TopicBack
            from .service import ServiceBack
            from .param import ParamBack
            from .ros_interface import RosInterface
            from .pyros_ros import PyrosROS

        # we return a relay of imported names, accessible the same way a direct import would be.
        return _PyrosSetup(TopicBack, ServiceBack, ParamBack, RosInterface, PyrosROS)

delayed_import = _PyrosSetup.delayed_import
delayed_import_auto = _PyrosSetup.delayed_import_auto

__all__ = [
    'delayed_import',
    'delayed_import_auto',
]

