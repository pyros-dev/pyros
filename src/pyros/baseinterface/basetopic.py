from __future__ import absolute_import
import abc


class BaseTopic(object):
    """
    BaseTopic is a class handling conversion from Python API to Topic call
    # TODO : Topic is a not a complete concept like service is.
    More research needed to find the complementary concept of the service concept:
    - high rate, (and potentially lossy ?).
    - opposite control flow
    - broadcasting ( sending to all instead of sending to one )
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, topic_name, topic_type):
        self.name = topic_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        self.msgtype = topic_type.msgtype

    @abc.abstractmethod
    def cleanup(self):
        return

    @abc.abstractmethod
    def publish(self, msg):
        return

    @abc.abstractmethod
    def get(self, num=0, consume=False):
        return

