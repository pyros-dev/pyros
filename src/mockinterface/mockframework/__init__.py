# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

import multiprocessing

# TODO: locked list class

#https://pymotw.com/2/multiprocessing/communication.html
#https://docs.python.org/2/library/multiprocessing.html#proxy-objects
manager = multiprocessing.Manager()

nodes = manager.list()

# Not clear if Lock is implemented or not :
# https://docs.python.org/2.7/library/multiprocessing.html#sharing-state-between-processes
# https://bugs.python.org/issue19864
# So we do it anyway to be safe
services_lock = manager.Lock()
services = manager.dict()

topics = manager.list()
params = manager.list()

from .node import Node, current_node
from .service import Service, discover


__all__ = [
    'current_node',
    'Node', 'nodes',
    'Service', 'services', 'services_lock', 'discover'
]
