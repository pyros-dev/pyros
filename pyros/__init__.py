# -*- coding: utf-8 -*-
from __future__ import absolute_import

import logging
import os

import sys

from ._version import __version__

# Here in case we want to put logging config in there.
# Useful if pyros started alone as simple "server node".
from . import config

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

# handling all import trickery to access dependent ROS modules here, our "entry point for import"
try:
    # this should work from python venv or from ROS environment already setup.
    import pkg_resources
    import pyzmp
except (pkg_resources.DistributionNotFound, ImportError):
    # Handling hybrid usecase : package built in a devel workspace with catkin, and used from normal python
    import pyros_setup  # this has to be in your python environment.

    # Lets use the system configuration already setup by pyros-setup
    pyros_setup.configurable_import().configure().activate()
    # This will use default pyros_setup.cfg. see http://flask.pocoo.org/docs/0.11/config/#instance-folders
    # also pyros_setup should create the file if it doesnt exist.

    _logger.warning("Modified Python sys.path {0}".format(sys.path))
    import pyzmp

from .exceptions import PyrosException
from .pyros_ctx_server import pyros_ctx
from .pyros_client import PyrosClient, PyrosServiceNotFound, PyrosServiceTimeout
from .pyros_ros import PyrosROS
from .pyros_mock import PyrosMock

__all__ = [
    '__version__',
    'PyrosException',
    'PyrosServiceNotFound',
    'PyrosServiceTimeout',
    'pyros_ctx',
    'PyrosClient',
    'PyrosROS',
    'PyrosMock',
]