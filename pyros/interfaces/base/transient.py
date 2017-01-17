from __future__ import absolute_import
from __future__ import print_function


class Transient(object):
    """
    Transient is a base class handling transient description.
    This transient can be a service, a param, a publisher, etc.
    This class aims at making this transient usable from python, in a common way for all implementations.
    It will be manipulated from TransientPool, to track when it goes up and down and when it should be interfaced.
    """

    def __init__(self, transient_name, transient_type=None, transient_endpoints=None):
        """
        Building a transient from external data
        :param transient_name:
        :param transient_type:
        :param transient_endpoints: a set containing multiple endpoints tuples (node_name, uri, ...)
        Note the endpoints can be specific to the implementation
        """

        self.name = transient_name
        self.type = transient_type
        self.endpoints = transient_endpoints or set()

