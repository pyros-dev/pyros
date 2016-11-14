from __future__ import absolute_import
from collections import namedtuple, deque

from ...baseinterface import TransientIf


class MockPublisher(TransientIf):
    """
    MockPublisher is a mock of the class handling conversion from Python API to Publisher call
    """

    def __init__(self, topic_name, topic_type):

        # getting the fullname to make sure we start with /
        topic_name = topic_name if topic_name.startswith('/') else '/' + topic_name

        topic_type = topic_type.msgtype

        super(MockPublisher, self).__init__(topic_name, topic_type)

    def cleanup(self):
        pass

    def publish(self, msg):
        # System should be set by system itself ( TODO : investigate proper way to inject dependency for Mock...)
        return self.system.transfer(msg, self.name)


