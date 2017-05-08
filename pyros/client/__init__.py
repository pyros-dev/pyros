# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function


import logging

"""
Hopefully this should endup in ros.__doc__
"""

# create logger
# TODO solve multiprocess logger problem(s)...
_logger = logging.getLogger(__name__)

# The client part should be the same for any kind of backend.
# The only requirement for this client is to be used in a python environment.
# Therefore it is not, and will not be, a namespace package.

# But we do need to try to load from ROS environment, in case loading from python environment fails
try:
    # early except to prevent unintentional workaround in all modules here for packages we depend on
    import pyros_interfaces_common
    # this is only useful when these dependent packages are in a catkin/ROS workspace
    # TODO : detect if we are in a catkin workspace / following a ROS workflow ?
    # TODO : probably better than activate if failure occurs...
    # the main issue is for hybrid workflows...
except ImportError:
    _logger.warning("loading pyros_setup and configuring your ROS environment")
    import pyros_setup
    # This will load the pyros_setup configuration from the environment
    pyros_setup.configurable_import().configure().activate()
    # validate we can load ROS modules. Note other variables (like ROS_PACKAGE_PATH) should also be available.
    import pyros_interfaces_common


from .client import PyrosClient

__all__ = [
    'PyrosClient',
]




