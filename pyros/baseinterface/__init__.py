# -*- coding: utf-8 -*-
# This python package is handling all MOCK communication for rostful-node.
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
#
# If possible these should also be base classes for all the possible multiprocess framework interfaces
# available in pyros
#
# Mock interface is provided with the package, because it may help client libraries to implement their own tests
#
from __future__ import absolute_import

from .baseinterface import BaseInterface, DiffTuple
from .baseservice import BaseService
from .basetopic import BaseTopic

__all__ = [
    'BaseService',
    'BaseTopic',
    'DiffTuple',
    'BaseInterface',
]
