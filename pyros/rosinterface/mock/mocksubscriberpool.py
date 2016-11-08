from __future__ import absolute_import

from contextlib import contextmanager

from ...baseinterface import TransientIfPool
from .mocksystem import (
    topics_available_remote, topics_available_type_remote,
)

from .mocksubscriber import MockSubscriber


class MockSubscriberPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, topics=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        super(MockSubscriberPool, self).__init__(topics)

    # mock functions that simulate/mock similar interface than what is found on multiprocess framework supported
    # We should try our best to go for the lowest common denominator here
    # TOPICS
    def get_transients_available(self):  # function returning all topics available on the system
        return self.available

    def transient_type_resolver(self, topic_name):  # function resolving the type of a topic
        tpc = self.available.get(topic_name)
        return tpc  # None is returned if not found

    def TransientMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return MockSubscriber(topic_name, topic_type, *args, **kwargs)

    def TransientCleaner(self, topic):  # the topic class implementation
        return topic.cleanup()

    def update(self):
        for t in topics_available_remote.keys():  # explicit keys() call in case we work through a dictproxy
            # if topics_available_remote.get(t, 0) > 0:  # currently not needed
                self.available[t] = topics_available_type_remote.get(t)

        dt = self.transient_change_detect()

        return dt

TransientIfPool.register(MockSubscriberPool)
