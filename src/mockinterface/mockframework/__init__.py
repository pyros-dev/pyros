# -*- coding: utf-8 -*-
# This python package is implementing a very simple multiprocess framework
# The point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

import multiprocessing

# TODO: locked list class

nodes_lock = multiprocessing.Lock()
nodes = []

services_lock = multiprocessing.Lock()
services = []

topics_lock = multiprocessing.Lock()
topics = []

params_lock = multiprocessing.Lock()
params = []

from .node import Node, current_node
from .service import Service


__all__ = [
    'current_node',
    'Node', 'nodes',
    'Service', 'services',
]
