# -*- coding: utf-8 -*-
#This python package is handling all ROS related communication for rostful-node.
from __future__ import absolute_import

"""
Hopefully this should endup in rosinterface.__doc__
"""

# This should be found (from python dependencies at least)
import pyros_setup

from . import pyros_setup_config

# Here we use pyros_setup configuration to setup ROS if needed before using any of our modules
# This is much simpler since we don't need any other configuration at import time.
# Extra Pyros configuration can be passed at runtime directly to the interface.

try:
    import rospy  # early except to prevent unintentional workaround in all modules here
    from .topic import TopicBack
    from .service import ServiceBack
    from .param import ParamBack
    from .ros_interface import RosInterface
except ImportError:
    # This will load the pyros_setup configuration from a local object
    pyros_setup.configurable_import(instance_relative_config=False).configure(pyros_setup_config).activate()
    import rospy
    from .topic import TopicBack
    from .service import ServiceBack
    from .param import ParamBack
    from .ros_interface import RosInterface


__all__ = [
    'TopicBack',
    'ServiceBack',
    'ParamBack',
    'RosInterface',
]