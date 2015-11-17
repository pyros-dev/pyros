# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

import multiprocessing

from .master import manager
from .exceptions import UnknownServiceException, UnknownRequestTypeException, UnknownResponseTypeException
from .node import Node, current_node
from .service import Service, services, discover

# TODO: locked list class



nodes = manager.list()


topics = manager.list()
params = manager.list()



__all__ = [
    'current_node',
    'UnknownServiceException', 'UnknownRequestTypeException', 'UnknownResponseTypeException',
    'Node', 'nodes',
    'Service', 'services', 'discover'
]
