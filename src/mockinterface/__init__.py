# -*- coding: utf-8 -*-
# This python package is handling all MOCK communication for rostful-node.
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, in one process, without having to run a ROS system.
from __future__ import absolute_import

from .mocktopic import MockTopic, statusecho_topic
#from .mockaction import MockAction
from .mockservice import MockService, statusecho_service
from .mockparam import MockParam

from .mockmessage import (
    extract_values,
    populate_instance,
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
    'statusecho_topic',
    'statusecho_service',
]
