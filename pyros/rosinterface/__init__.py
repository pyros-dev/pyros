# -*- coding: utf-8 -*-
#This python package is handling all ROS related communication for rostful-node.
from __future__ import absolute_import

import logging

"""
Hopefully this should endup in rosinterface.__doc__
"""

# create logger
# TODO solve multiprocess logger problem(s)...
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

# Here we use pyros_setup configuration to setup ROS if needed before using any of our modules
# This is much simpler since we don't need any other configuration at import time.
# Extra Pyros configuration can be passed at runtime directly to the interface.

try:
    from .api import rospy_safe  # early except to prevent unintentional workaround in all modules here
    # We ideally should add all dependencies imported by the modules in this subpackage...
    # We should be fine here when running from deb.
    # But when running from python we have to except here to get environment setup properly in child process.
except ImportError:
    # This should be found from python dependencies. But not needed when running from deb.
    # Here should be the first time we setup ros variable in pyros package,
    # in the child process, since the child (and only the child) needs to know about ROS.
    _logger.warning("loading pyros_setup and configuring your ROS environment")
    import pyros_setup
    # This will load the pyros_setup configuration from the environment
    pyros_setup.configurable_import().configure().activate()
    # validate we can load ROS modules. Note other variables (like ROS_PACKAGE_PATH) should also be available.
    from .api import rospy_safe

from .service_if import ServiceBack
from .param_if import ParamBack
from .param_if_pool import RosParamIfPool
from .service_if_pool import RosServiceIfPool
from .subscriber_if_pool import RosSubscriberIfPool
from .publisher_if_pool import RosPublisherIfPool
from .baseinterface import BaseInterface
from .ros_interface import RosInterface


__all__ = [
    'TopicBack',
    'ServiceBack',
    'ParamBack',
    'RosInterface',
]