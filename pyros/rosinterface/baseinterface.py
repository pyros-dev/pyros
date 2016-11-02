from __future__ import absolute_import

import logging
import six
import sys
import threading
import collections
import re
import abc
from functools import partial

from ..baseinterface.regex_tools import regexes_match_sublist
from ..baseinterface import TransientIfPool
from .param_if_pool import RosParamIfPool
from .service_if_pool import RosServiceIfPool
from .topic_if_pool import RosTopicIfPool


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

    def get_svc_list(self):  # function returning all services available on the system
        return self.services_pool.get_transients_available()

    def service_type_resolver(self, service_name):  # function resolving the type of a service
        """
        :param service_name: the name of the service
        :return: returns None if the type cannot be found. Properly except in all other unexpected events.
        """
        return self.services_pool.transient_type_resolver(service_name)

    def ServiceMaker(self, service_name, service_type, *args, **kwargs):  # the service class implementation
        return self.services_pool.TransientMaker(service_name, service_type, *args, **kwargs)

    def ServiceCleaner(self, service):  # the service class implementation
        return self.services_pool.TransientCleaner(service)

    # Abstract methods to override for topics ("pub/sub type" communication channel)
    def get_topic_list(self):  # function returning all topics available on the system
        return self.topics_pool.get_transients_available()

    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        """
        :param topic_name: the name of the topic
        :return: returns None if the topic cannot be found. Properly except in all other unexpected events.
        """
        return self.topics_pool.transient_type_resolver(topic_name)

    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return self.topics_pool.TransientMaker(topic_name, topic_type, *args, **kwargs)

    def TopicCleaner(self, topic):  # the topic class implementation
        return self.topics_pool.TransientCleaner(topic)

    # Abstract methods to override for params ( "global" get/set objects )
    def get_param_list(self):  # function returning all topics available on the system
        return self.params_pool.get_transients_available()

    def param_type_resolver(self, param_name):  # function resolving the type of a topic
        return self.params_pool.transient_type_resolver(param_name)

    def ParamMaker(self, param_name, param_type, *args, **kwargs):  # the param class implementation
        return self.params_pool.TransientMaker(param_name, param_type, *args, **kwargs)

    def ParamCleaner(self, param):  # the param class implementation
        return self.params_pool.TransientCleaner(param)

    def __init__(self, services, topics, params):
        """
        Initializes the interface instance, to expose services, topics, and params
        """
        # Current transients exposed, i.e. those which are
        # active in the system.

        self.params_pool = RosParamIfPool(params)
        self.services_pool = RosServiceIfPool(services)
        self.topics_pool = RosTopicIfPool(topics)

    # REQUESTED
    @property
    def services_args(self):
        return self.services_pool.transients_args

    @property
    def topics_args(self):
        return self.topics_pool.transients_args

    @property
    def params_args(self):
        return self.params_pool.transients_args

    # AVAILABLE
    @property
    def services_available(self):
        return self.services_pool.available

    @property
    def topics_available(self):
        return self.topics_pool.available

    @property
    def params_available(self):
        return self.params_pool.available

    # INTERFACED
    @property
    def services(self):
        return self.services_pool.transients

    @property
    def topics(self):
        return self.topics_pool.transients

    @property
    def params(self):
        return self.params_pool.transients

    # EXPOSE
    def expose_services(self, svc_regex):
        return self.services_pool.expose_transients_regex(svc_regex)

    def expose_topics(self, tpc_regex):
        return self.topics_pool.expose_transients_regex(tpc_regex)

    def expose_params(self, prm_regex):
        return self.params_pool.expose_transients_regex(prm_regex)


    # def update_on_diff(self, services_dt, topics_dt, params_dt):
    #
    #     sdt = self.update_services(add_names=[m for m in regexes_match_sublist(self.services_args, services_dt.added)],
    #                                remove_names=services_dt.removed
    #                                )
    #     tdt = self.update_topics(add_names=[m for m in regexes_match_sublist(self.topics_args, topics_dt.added)],
    #                              remove_names=topics_dt.removed
    #                              )
    #     pdt = self.update_params(add_names=[m for m in regexes_match_sublist(self.params_args, params_dt.added)],
    #                              remove_names=params_dt.removed
    #                              )
    #
    #     return DiffTuple(
    #         added=sdt.added+tdt.added+pdt.added,
    #         removed=sdt.removed+tdt.removed+pdt.removed
    #     )
    #
    # def update(self):
    #     """
    #     :return: the difference between the transients recently added/removed
    #     """
    #     sdt = self.services_change_detect()
    #     tdt = self.topics_change_detect()
    #     pdt = self.params_change_detect()
    #
    #     return DiffTuple(
    #         added=sdt.added+tdt.added+pdt.added,
    #         removed=sdt.removed+tdt.removed+pdt.removed
    #     )

    # TODO : "wait_for_it" methods that waits for hte detection of a topic/service on the system
    # TODO : Should return a future so use can decide to wait on it or not
    # TODO : Maybe similar to a async_detect ( hooked up to the detected transient, not the exposed ones )
    # TODO : Exposed interface is for direct control flow => async not really needed
    # TODO : Detect/Update interface is inversed control flow ( from update loop ) => Needed
