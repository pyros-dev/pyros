from __future__ import absolute_import

import time
from collections import deque

from .api import rospy_safe as rospy
from .message_conversion import extract_values, FieldTypeMismatchException
from .poolparam import PoolParam
from .topicbase import TopicBase


class PublisherBackTimeout(Exception):
    pass


class PublisherBack(TopicBase):
    """
    TopicBack is the class handling conversion from Python to ROS Topic
    Requirement : Only one topicBack per actual ROS Topic.
    Since we connect to an already existing ros topic, our number of connections should never drop under 1
    """

    # We need some kind of instance count here since system state returns only one node instance
    # as publisher for multiple publisher in this process.
    #
    # This is used in ros_interface update for added puba/subs to determine
    # if we previously started publishing / subscribing to this topic
    # usually the count will be just 1, but more is possible during tests
    #
    # This is also used in ros_interface update for removed pubs/subs to determine
    # if we previously stopped publishing / subscribing to this topic
    # usually the count will be just 1, but more is possible during tests

    pool = PoolParam(rospy.Subscriber, "subscribers")

    def __init__(self, topic_name, topic_type, msg_queue_size=1):
        # Parent class will resolve/normalize topic_name
        super(PublisherBack, self).__init__(topic_name, topic_type)

        rospy.loginfo(
            rospy.get_name() + " Pyros.rosinterface : Adding subscriber interface {name} {typename}".format(
                name=self.name, typename=self.rostype))

        # this message queue should be ready before we setup the callback
        # TODO : change to a proper Queue
        self.msg = deque([], msg_queue_size)

        self.topic = self.pool.acquire(self.name, self.rostype, self.topic_callback, queue_size=1)

        self.empty_cb = None

        # Make sure we get at least one connection before returning
        start = time.time()
        timeout = 1
        while time.time() - start < timeout and len(self.pool.get_impl_connections(self.name)) < 1:
            # print("subscribers connected : {0}".format(self.pool.get_impl_connections(self.name)))
            time.sleep(0.2)

        if self.pool.get_impl_connections(self.name) < 1:
            raise PublisherBackTimeout

    def cleanup(self):
        """
        Launched when we want to whithhold this interface instance
        :return:
        """

        # TODO : should we do this in del method instead ? to allow reuse until garbage collection actually happens...
        rospy.loginfo(
            rospy.get_name() + " Pyros.rosinterface : Removing subscriber interface {name} {typename}".format(
                name=self.name, typename=self.rostype))

        self.pool.release(self.topic)

        super(PublisherBack, self).cleanup()

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """
        d = super(PublisherBack, self).asdict()
        d['publishers'] = self.topic.impl.get_stats_info()
        return d

    def get(self, num=0, consume=False):
        if not self.msg:
            return None
        # TODO : implement a way to have "plug and play" behaviors (some can be "all, paged, FIFO, etc." with custom code that can be insterted here...)
        res = None
        #TODO : implement returning multiple messages ( paging/offset like for long REST requests )
        if consume:
            res = self.msg.popleft()
            if 0 == len(self.msg) and self.empty_cb:
                self.empty_cb()
                #TODO : CHECK that we can survive here even if we get dropped from the topic list
        else:
            res = self.msg[0]
            try:
                res = extract_values(res)
            except FieldTypeMismatchException as e:
                rospy.logerr("[{name}] : field type mismatch {e}".format(name=__name__, e=e))
                raise
                # TODO : reraise a topic exception ?
        return res

    #returns the number of unread message
    def unread(self):
        return len(self.msg)

    def set_empty_callback(self, cb):
        self.empty_cb = cb

    def topic_callback(self, msg):
        # TODO : we are duplicating the queue behavior that is already in rospy... Is there a better way ?
        self.msg.appendleft(msg)

