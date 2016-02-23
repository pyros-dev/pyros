# -*- coding: utf-8 -*-
# This python package is implementing a simple multiprocess framework.
# Although we need to keep it simple, and minimalist, it should provide everything needed
# for efficient multiprocess programming.
# The goal is to make is as simple as possible to build resilient parallel software, like with functional programming,
# yet to be in total control of what each process does.
# Possible target applications are multi process systems ( ROS, HPC, celery, Neural Networks, etc. )
#

# In rostful-node, the point of it is to be able to fully tests the multiprocess behavior,
#     in pure python, without having to run a ROS system.
from __future__ import absolute_import

from .exceptions import UnknownServiceException, UnknownRequestTypeException, UnknownResponseTypeException
from .master import manager
from .node import Node, current_node
from .service import Service, services, discover, ServiceCallTimeout


topics = manager.list()
params = manager.list()


__all__ = [
    'UnknownServiceException', 'UnknownRequestTypeException', 'UnknownResponseTypeException',
    'Node', 'nodes', 'current_node',
    'Service', 'services', 'discover', 'ServiceCallTimeout'
]
