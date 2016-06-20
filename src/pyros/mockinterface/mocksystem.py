# -*- coding: utf-8 -*-
from __future__ import absolute_import

#
# Simulating an external environment
#

from contextlib import contextmanager
import multiprocessing

# This manager process provides a mock implementation while allowing multiprocess access to it
# Especially useful for test client process.
mock_manager = multiprocessing.Manager()

# These list what is available on the Mock implementation.
# They are accessible directly for tests who want to simulate multiprocess communication framework changes.
services_available_remote = mock_manager.list()
services_available_type_remote = mock_manager.dict()
topics_available_remote = mock_manager.list()
topics_available_type_remote = mock_manager.dict()
params_available_remote = mock_manager.list()
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
def mock_topic_remote(topic_name, topic_type):
    print(" -> Mock Topic {topic_name} appear".format(**locals()))
    topics_available_remote.append(topic_name)  # Service appears
    topics_available_type_remote[topic_name] = topic_type
    yield
    topics_available_remote.remove(topic_name)  # Service disappears
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
