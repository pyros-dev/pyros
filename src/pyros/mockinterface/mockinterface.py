from __future__ import absolute_import

import logging
import abc
from pyros.baseinterface import BaseInterface

from .mockservice import MockService
from .mocktopic import MockTopic
from .mockparam import MockParam

import unicodedata


class MockInterface(BaseInterface):
    """
    MockInterface.
    """
    def __init__(self, services=None, topics=None, params=None):
        # Current mock implementation of services, topics and params
        if services is None:
            services = []
        if topics is None:
            topics = []
        if params is None:
            params = []
        # These list what is available on the Mock implementation.
        # They are accessible directly for tests who want to simulate multiprocess communication framework changes.
        self.services_available = []
        self.services_available_type = {}
        self.topics_available = []
        self.topics_available_type = {}
        self.params_available = []
        self.params_available_type = {}
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_list() )
        super(MockInterface, self).__init__(services, topics, params)

    # mock functions that simulate/mock similar interface than what is found on multiprocess framework supported
    # We should try our best to go for the lowest common denominator here
    # SERVICES
    def get_svc_list(self):  # function returning all services available on the system
        return self.services_available

    def service_type_resolver(self, service_name):  # function resolving the type of a service
        return self.services_available_type.get(service_name, None)

    def ServiceMaker(self, service_name, service_type, *args, **kwargs):  # the service class implementation
        return MockService(service_name, service_type, *args, **kwargs)

    def ServiceCleaner(self, service):  # the service class cleanup implementation
        return service.cleanup()

    def mock_service_appear(self, svc_name, svc_type):
        self.services_available.append(svc_name)  # Service appears
        self.services_available_type[svc_name] = svc_type

    def mock_service_disappear(self, svc_name):
        self.services_available.remove(svc_name)  # Service disappears
        self.services_available_type.pop(svc_name)

    # TOPICS
    def get_topic_list(self):  # function returning all topics available on the system
        return self.topics_available

    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        return self.topics_available_type.get(topic_name, None)

    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return MockTopic(topic_name, topic_type, *args, **kwargs)

    def TopicCleaner(self, topic):  # the topic class implementation
        return topic.cleanup()

    def mock_topic_appear(self, topic_name, topic_type):
        self.topics_available.append(topic_name)  # Service appears
        self.topics_available_type[topic_name] = topic_type

    def mock_topic_disappear(self, topic_name):
        self.topics_available.remove(topic_name)  # Service disappears
        self.topics_available_type.pop(topic_name)

    def get_param_list(self):  # function returning all params available on the system
        return self.params_available

    def param_type_resolver(self, param_name):  # function resolving the type of a param
        return self.params_available_type.get(param_name, None)

    def ParamMaker(self, param_name, param_type, *args, **kwargs):  # the param class implementation
        return MockParam(param_name, param_type, *args, **kwargs)

    def ParamCleaner(self, param):  # the param class implementation
        return param.cleanup()

    def mock_param_appear(self, param_name, param_type):
        self.params_available.append(param_name)  # Param appears
        self.params_available_type[param_name] = param_type

    def mock_param_disappear(self, param_name):
        self.params_available.remove(param_name)  # Param disappears
        self.params_available_type.pop(param_name)

BaseInterface.register(MockInterface)
