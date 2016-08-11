# -*- coding: utf-8 -*-
from __future__ import absolute_import

import logging
import os

import sys

from ._version import __version__


# handling all import trickery to access dependent ROS modules here, our "entry point for import"
try:
    # this should work from python venv or from ROS environment already setup.
    import pkg_resources
    import pyzmp
    import pyros_utils
except (pkg_resources.DistributionNotFound, ImportError):
    # Handling hybrid usecase : package built in a devel workspace with catkin, and used from normal python
    import pyros_setup  # this has to be in your python environment.
    # This will setup a python environment dynamically (useful for ROS systems)
    pyros_setup.configurable_import(instance_relative_config=False).configure({
        # since we didn't find pyzmp in our current environment,
        # lets assume it is in our devel workspace, and it has not been setup yet.
        'WORKSPACES': [
            os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'devel')),
        ],
        'DISTRO': 'indigo',
    }).activate()
    logging.warning("Modified Python sys.path {0}".format(sys.path))
    import pyzmp
    import pyros_utils

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