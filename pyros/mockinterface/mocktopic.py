from __future__ import absolute_import
from collections import namedtuple, deque

TopicType = namedtuple("TopicType", "msgtype")

statusecho_topic = TopicType("StatusMsg")


class MockTopic(object):
    """
    MockTopic is a mock of the class handling conversion from Python API to Topic call
    """

    # Mock Implementation
    # Intra-process inter-thread communication channel : a simple class variable
    _msg_content = None

    def __init__(self, topic_name, topic_type, queue_size=1):

        self.name = topic_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        self.msgtype = topic_type.msgtype
        self.msg = deque([], queue_size)

        self.empty_cb = None

    def cleanup(self):
        pass

    def publish(self, msg):
        self._msg_content = msg
        self.topic_callback(msg)
        return True

    def get(self, num=0, consume=False):
        if not self.msg:
            return None

        res = None
        #TODO : implement returning multiple messages
        if consume:
            res = self.msg.popleft()
            if 0 == len(self.msg) and self.empty_cb:
                self.empty_cb()
                #TODO : CHECK that we can survive here even if we get dropped from the topic list
        else:
            res = self.msg[0]

        return res

    #returns the number of unread message
    def unread(self):
        return len(self.msg)

    def set_empty_callback(self, cb):
        self.empty_cb = cb

    # potentially called by a different thread : keep it miminal, just queue things.
    def topic_callback(self, msg):
        self.msg.appendleft(msg)

