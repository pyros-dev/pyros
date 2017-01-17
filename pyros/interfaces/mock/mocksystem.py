# -*- coding: utf-8 -*-
from __future__ import absolute_import

#
# Simulating an external environment
#

from contextlib import contextmanager
import multiprocessing
from collections import namedtuple


TopicType = namedtuple("TopicType", "msgtype")

statusecho_topic = TopicType("StatusMsg")


# This manager process provides a mockinterface implementation while allowing multiprocess access to it
# Especially useful for test client process.
mock_manager = multiprocessing.Manager()

# These list what is available on the Mock implementation.
# They are accessible directly for tests who want to simulate multiprocess communication framework changes.
services_available_remote = mock_manager.list()  # Service is unique
services_available_type_remote = mock_manager.dict()

# Same type for pub and subs
topics_available_remote = mock_manager.dict()  # instance Counter ( Potentially N Pubs + M Subs )
topics_available_type_remote = mock_manager.dict()

params_available_remote = mock_manager.list()  # Param is unique
params_available_type_remote = mock_manager.dict()


@contextmanager
def mock_service_remote(svc_name, svc_type):
    print(" -> Mock Service {svc_name} appear".format(**locals()))
    services_available_remote.append(svc_name)  # Service appears
    services_available_type_remote[svc_name] = svc_type
    yield
    services_available_remote.remove(svc_name)
    services_available_type_remote.pop(svc_name)
    print(" -> Mock Service {svc_name} disappear".format(**locals()))


@contextmanager
def mock_publisher_remote(topic_name, topic_type):
    print(" -> Mock Topic {topic_name} appear".format(**locals()))
    topics_available_remote[topic_name] = topics_available_remote.get(topic_name, 0) + 1  # Publisher appears
    if topic_name in topics_available_type_remote:
        assert(topics_available_type_remote[topic_name] == topic_type)
    else:
        topics_available_type_remote[topic_name] = topic_type
    yield
    topics_available_remote[topic_name] = topics_available_remote.get(topic_name, 0) - 1  # Publisher disappears
    if topics_available_remote.get(topic_name, 0) == 0:
        topics_available_remote.pop(topic_name)
        topics_available_type_remote.pop(topic_name)
    print(" -> Mock Topic {topic_name} disappear".format(**locals()))

@contextmanager
def mock_subscriber_remote(topic_name, topic_type):
    print(" -> Mock Topic {topic_name} appear".format(**locals()))
    topics_available_remote[topic_name] = topics_available_remote.get(topic_name, 0) + 1  # Subscriber appears
    if topic_name in topics_available_type_remote:
        assert(topics_available_type_remote[topic_name] == topic_type)
    else:
        topics_available_type_remote[topic_name] = topic_type
    yield
    topics_available_remote[topic_name] = topics_available_remote.get(topic_name, 0) - 1  # Subscriber disappears
    if topics_available_remote.get(topic_name, 0) == 0:
        topics_available_remote.pop(topic_name)
        topics_available_type_remote.pop(topic_name)
    print(" -> Mock Topic {topic_name} disappear".format(**locals()))

@contextmanager
def mock_param_remote(param_name, param_type):
    print(" -> Mock Param {param_name} appear".format(**locals()))
    params_available_remote.append(param_name)  # Param appears
    params_available_type_remote[param_name] = param_type
    yield
    params_available_remote.remove(param_name)  # Param disappears
    params_available_type_remote.pop(param_name)
    print(" -> Mock Param {param_name} disappear".format(**locals()))


from .mockpublisher import MockPublisher
from .mocksubscriber import MockSubscriber
from .mockservice import MockService
from .mockparam import MockParam


# If we prefer a in process class instance instead of a remote process
# BUT remote process is used by Pyros_Mock
# TODO : choose ONE !
# TODO : find a simpler way to insure isolation for Mock...
# TODO : investigate how to mix / reuse standard python multprocessing lib
# Check SyncManager http://stackoverflow.com/questions/19568232/how-to-iterate-over-a-dict-proxy-in-python

class MockSystem(object):
    services_available_remote = []  # Service is unique
    services_available_type_remote = {}

    # Same type for pub and subs
    topics_available_remote = {}  # instance Counter ( Potentially N Pubs + M Subs )
    topics_available_type_remote = {}

    params_available_remote = []  # Param is unique
    params_available_type_remote = {}

    # Mock Implementation
    # Intra-process inter-thread communication channel : a simple global variable
    _msg_subs = dict()

    @contextmanager
    def mock_service_remote(self, svc_name, svc_type):
        print(" -> Mock Service {svc_name} appear".format(**locals()))
        self.services_available_remote.append(svc_name)  # Service appears
        self.services_available_type_remote[svc_name] = svc_type
        yield
        self.services_available_remote.remove(svc_name)
        self.services_available_type_remote.pop(svc_name)
        print(" -> Mock Service {svc_name} disappear".format(**locals()))

    @contextmanager
    def mock_publisher_remote(self, topic_name, topic_type):
        print(" -> Mock Topic {topic_name} appear".format(**locals()))
        self.topics_available_remote[topic_name] = self.topics_available_remote.get(topic_name, 0) + 1  # Publisher appears
        if topic_name in self.topics_available_type_remote:
            assert (self.topics_available_type_remote[topic_name] == topic_type)
        else:
            self.topics_available_type_remote[topic_name] = topic_type
        yield
        self.topics_available_remote[topic_name] = self.topics_available_remote.get(topic_name, 0) - 1  # Publisher disappears
        if self.topics_available_remote.get(topic_name, 0) == 0:
            self.topics_available_remote.pop(topic_name)
            topics_available_type_remote.pop(topic_name)
        print(" -> Mock Topic {topic_name} disappear".format(**locals()))

    @contextmanager
    def mock_subscriber_remote(self, topic_name, topic_type):
        print(" -> Mock Topic {topic_name} appear".format(**locals()))
        self.topics_available_remote.append(topic_name)  # Service appears
        self.topics_available_type_remote[topic_name] = topic_type
        yield
        self.topics_available_remote.remove(topic_name)  # Service disappears
        self.topics_available_type_remote.pop(topic_name)
        print(" -> Mock Topic {topic_name} disappear".format(**locals()))

    @contextmanager
    def mock_param_remote(self, param_name, param_type):
        print(" -> Mock Param {param_name} appear".format(**locals()))
        self.params_available_remote.append(param_name)  # Param appears
        self.params_available_type_remote[param_name] = param_type
        yield
        self.params_available_remote.remove(param_name)  # Param disappears
        self.params_available_type_remote.pop(param_name)
        print(" -> Mock Param {param_name} disappear".format(**locals()))

    # TODO : define a better API (more similar to rospy / ROS interfaces...)
    # TODO : link with .api module somehow...
    def create_publisher(self, topic_name, topic_type):
        pub = MockPublisher(topic_name=topic_name, topic_type=topic_type)
        pub.system = self  # to enable topic comm
        return pub

    def create_subscriber(self, topic_name, topic_type):
        sub = MockSubscriber(topic_name=topic_name, topic_type=topic_type)
        self._msg_subs[sub.name] = self._msg_subs.get(sub.name, []) + [sub]
        return sub

    def create_service(self, service_name, service_type):
        svc = MockService(service_name=service_name, service_type=service_type)
        return svc

    def create_parameter(self, param_name, param_type):
        prm = MockParam(param_name=param_name, param_type=param_type)
        return prm

    def transfer(self, msg, topic_name):
        for sub in self._msg_subs.get(topic_name, []):
            sub.topic_callback(msg)
        return True


__all__ = [

    'services_available_remote',
    'services_available_type_remote',
    'mock_service_remote',

    'topics_available_remote',
    'topics_available_type_remote',
    'mock_topic_remote'

    'params_available_remote',
    'params_available_type_remote',
    'mock_param_remote',
]
