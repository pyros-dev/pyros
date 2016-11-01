from __future__ import absolute_import

import logging
import six
import sys
import threading
import collections
import re
import abc
from functools import partial

from ..baseinterface import TransientIfPool

#TODO Entity Component System design for interface loop. cf https://pypi.python.org/pypi/esper (py3 + py2 in fork)
# Entities are transients (ex : for ROS : pubs, subs, svcs, params, and more can be added),
# Systems store logic about when/how a transient should be represented in the interface.
# GOALS : clarity, testability and flexibility
class BaseInterface(object):

    """
    BaseInterface.
    Assumption : we only deal with absolute names here. The users should resolve them
    """
    __metaclass__ = abc.ABCMeta


    # Abstract methods to override for services ("RPC / request / call type" communication channel)
    @abc.abstractmethod
    def get_svc_list(self):  # function returning all services available on the system
        return

    @abc.abstractmethod
    def service_type_resolver(self, service_name):  # function resolving the type of a service
        """
        :param service_name: the name of the service
        :return: returns None if the type cannot be found. Properly except in all other unexpected events.
        """
        return

    @abc.abstractmethod
    def ServiceMaker(self, service_name, service_type, *args, **kwargs):  # the service class implementation
        return

    @abc.abstractmethod
    def ServiceCleaner(self, service):  # the service class implementation
        return

    # Abstract methods to override for topics ("pub/sub type" communication channel)
    @abc.abstractmethod
    def get_topic_list(self):  # function returning all topics available on the system
        return

    @abc.abstractmethod
    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        """
        :param topic_name: the name of the topic
        :return: returns None if the topic cannot be found. Properly except in all other unexpected events.
        """
        return

    @abc.abstractmethod
    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return

    @abc.abstractmethod
    def TopicCleaner(self, topic):  # the topic class implementation
        return

    # Abstract methods to override for params ( "global" get/set objects )
    @abc.abstractmethod
    def get_param_list(self):  # function returning all topics available on the system
        return

    @abc.abstractmethod
    def param_type_resolver(self, param_name):  # function resolving the type of a topic
        return

    @abc.abstractmethod
    def ParamMaker(self, param_name, param_type, *args, **kwargs):  # the topic class implementation
        return

    @abc.abstractmethod
    def ParamCleaner(self, param):  # the topic class implementation
        return

    def __init__(self, services, topics, params):
        """
        Initializes the interface instance, to expose services, topics, and params
        """
        # Current transients exposed, i.e. those which are
        # active in the system.
        self.services_pool = TransientIfPool(services, transients_desc="services")
        self.topics_pool = TransientIfPool(topics, transients_desc="topics")
        self.params_pool = TransientIfPool(params, transients_desc="params")


    def update_on_diff(self, services_dt, topics_dt, params_dt):

        sdt = self.update_services(add_names=[m for m in regexes_match_sublist(self.services_args, services_dt.added)],
                                   remove_names=services_dt.removed
                                   )
        tdt = self.update_topics(add_names=[m for m in regexes_match_sublist(self.topics_args, topics_dt.added)],
                                 remove_names=topics_dt.removed
                                 )
        pdt = self.update_params(add_names=[m for m in regexes_match_sublist(self.params_args, params_dt.added)],
                                 remove_names=params_dt.removed
                                 )

        return DiffTuple(
            added=sdt.added+tdt.added+pdt.added,
            removed=sdt.removed+tdt.removed+pdt.removed
        )

    def update(self):
        """
        :return: the difference between the transients recently added/removed
        """
        sdt = self.services_change_detect()
        tdt = self.topics_change_detect()
        pdt = self.params_change_detect()

        return DiffTuple(
            added=sdt.added+tdt.added+pdt.added,
            removed=sdt.removed+tdt.removed+pdt.removed
        )

    # TODO : "wait_for_it" methods that waits for hte detection of a topic/service on the system
    # TODO : Should return a future so use can decide to wait on it or not
    # TODO : Maybe similar to a async_detect ( hooked up to the detected transient, not the exposed ones )
    # TODO : Exposed interface is for direct control flow => async not really needed
    # TODO : Detect/Update interface is inversed control flow ( from update loop ) => Needed
