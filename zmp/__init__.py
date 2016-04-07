# -*- coding: utf-8 -*-
# This python package is implementing a simple multiprocess framework.
# Although we need to keep it simple, and minimalist, it should provide everything needed
# for efficient multiprocess programming.
# The goal is to make is as simple as possible to build resilient parallel software, like with functional programming,
# yet to be in total control of what each process does.
# Possible target applications are multi process systems ( ROS, HPC, celery, Neural Networks, etc. )
#

# Concepts to keep in mind :
# - services (request based - pull oriented model - sync or async - Ex: web, RPC, etc.)
# - topic/feeds ( message based - push oriented model - sync or async - Ex: Pub/Sub, XMPP, etc.)
# - echo nodes ( only echo back messages or requests as is)
# - forward nodes ( only forward messages or requests as is)
# - shadows ( allow switching from push model to pull model via an intermediate that store data - cf. Amazon IoT architecture design )
# - ghost ( allow switching from pull model to push model via an intermediate that broadcast estimated data )
# ghost service depends on shadow service, shadow feed depends on ghost feed... kind of.
# Lots of explicit details are needed in implementation ( frequency of publishing, paramters of requests, variation of data/response, etc.)
# Estimating/simulating all that is not trivial, but that's very likely the gist of the "learning" behavior of neuronal networks.

# In pyros, the point of it is to be able to fully tests the multiprocess behavior,
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
