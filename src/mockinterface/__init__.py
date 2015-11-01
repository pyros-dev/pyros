# -*- coding: utf-8 -*-
# This python package is handling all MOCK communication for rostful-node.
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

#from .mocktopic import MockTopic
#from .mockaction import MockAction
#from .mockservice import MockService
#from .mockparam import MockParam

from .mockmessage import (
    extract_values as extract_msg,
    populate_instance as populate_msg,
    FieldTypeMismatchException,
    NonexistentFieldException,
    StatusMsg,
)

__all__ = [
    'MockTopic',
    'MockService',
    'MockParam',
    'MockAction',
    'extract_msg',
    'populate_msg',
    'FieldTypeMismatchException',
    'NonexistentFieldException',
    'StatusMsg',
]
