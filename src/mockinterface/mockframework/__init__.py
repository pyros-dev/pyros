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
services = manager.dict()
topics = manager.list()
params = manager.list()

from .node import Node, current_node
from .service import Service, discover


__all__ = [
    'current_node',
    'Node', 'nodes',
    'Service', 'services', 'discover'
]
