from __future__ import absolute_import

import time

import roslib
import rospy

from importlib import import_module
from collections import deque, OrderedDict


from .message_conversion import get_msg, get_msg_dict, populate_instance, extract_values, FieldTypeMismatchException

from .publisher import PublisherBack
from .subscriber import SubscriberBack


class TopicBackTimeout(Exception):
    pass


# Maybe have a global method that generate a context manager to interface with it...
class TopicTuple(object):
    def __init__(self, name, type, endpoints):
        self.name = name
        self.type = type
        self.endpoints = endpoints
# Note : for topic the connection endpoint is important.
# We can have multiple subscribers and publishers, from different node.
# We need to know if we can drop our interface when we receive a difference ( only some pub|sub lost )
# => we need to track endpoints
# TODO: make that the pickled representation of TopicBack (check asdict())


class TopicBack(object):
    """
    TopicBack is the class handling conversion from Python to ROS Topic
    This is a basic temporary implementation to provide same API as previous 0.2 versions
    Goal is to provide different configuration for publishers and subscribers
    => this should disappear with pyros 0.3
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


    # TODO: maybe contextmanager to make sure unregister always happens if we create it ?
    # Should be fine since hte interface is in loop...
    # this might help keep track of the interface created pub/sub, and help keep things tidy...

    @staticmethod
    def _create_pub(name, rostype, *args, **kwargs):
        """
        Creating a publisher and adding it to the pub instance count.
        Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
        :return: the ros publisher
        """
        return PublisherBack._create(name, rostype, *args, **kwargs)


    @staticmethod
    def _remove_pub(pub):
        """
        Removing a publisher and substracting it from the pub instance count.
        :return: None
        """
        return PublisherBack._remove(pub)

    @staticmethod
    def _create_sub(name, rostype, topic_callback, *args, **kwargs):
        """
        Creating a subscriber and adding it to the pub instance count.
        Static and Functional style so we can call it from anywhere (tests).
        Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
        :return: the subscriber
        """
        return SubscriberBack._create(name, rostype, topic_callback, *args, **kwargs)

    @staticmethod
    def _remove_sub(sub):
        """
        Removing a subscriber and substracting it from the sub instance count.
        :return: None
        """
        return SubscriberBack._remove(sub)


    # We need this because we cannot really trust get_num_connections() (updated only after message is published)
    # USED !
    @staticmethod
    def get_impl_ref_count(name):
        """
        Return the number of internal references to that publisher implementation
        Useful to keep track of multiple instance of publisher in one process(for tests for examples)
        In all cases we have only one implementation (rospy), one interface (pyros), but external code in same process
        can add references, that we want to be counted as part of system, and not part of interface...
        TODO : test nodelets...
        :param name: name of the topic
        :return: impl.ref_count() for that topic
        """
        return PublisherBack.pool.get_impl_ref_count(name)

    def __init__(self, topic_name, topic_type, sub_queue_size=1, pub_start_timeout=2):
        # We need to create sub first
        self.if_sub = SubscriberBack(topic_name, topic_type, sub_queue_size)

        self.if_pub = PublisherBack(topic_name, topic_type)

        # Here making sure the publisher is actually connected
        # before returning to ensure RAII
        # Note : The sub should be immediately usable.
        start = time.time()
        timeout = pub_start_timeout
        # REMEMBER sub connections is a list of pubs and pub connections is the list of subs...
        while time.time() - start < timeout and (
            self.if_pub.topic.get_num_connections() < 1  # at least our own subscriber should connect here
        ):
            rospy.rostime.wallsleep(0.1)
        if not time.time() - start < timeout:
            raise TopicBackTimeout()
        # Note : we cannot do that in PublisherBack, because we do not know if a subscriber even exist in the system...

    def cleanup(self):
        """
        Launched when we want to whithhold this interface instance
        :return:
        """
        # cleanup pub and sub, so we can go through another create / remove cycle properly
        # Sub needs to be removed first if we want the pub removal to not throw errors
        self.if_sub.cleanup()
        # The pub is not going to detect the sub is gone, unless a message is send
        # which will refresh the connections list.
        # => we need to remove it without waiting
        self.if_pub.cleanup()

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """
        # simple merge for now...
        d = self.if_pub.asdict()
        d.update(self.if_sub.asdict())
        return d

    # TMP...
    @property
    def name(self):
        return self.if_pub.name

    # TMP...
    @staticmethod
    def get_all_interfaces():
        return PublisherBack.pool.get_all_interfaces()

    def publish(self, msg_content):
        """
        Publishes a message to the topic
        :return the actual message sent if one was sent, None if message couldn't be sent.
        """
        return self.if_pub.publish(msg_content=msg_content)

    def get(self, num=0, consume=False):
        return self.if_sub.get(num=num, consume=consume)

    #returns the number of unread message
    def unread(self):
        return self.if_sub.unread()

    def set_empty_callback(self, cb):
        return self.if_sub.unread()

    def topic_callback(self, msg):
        return self.if_sub.topic_callback(msg=msg)

