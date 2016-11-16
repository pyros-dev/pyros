from __future__ import absolute_import

import time

import roslib
import rospy

from importlib import import_module
from collections import deque, OrderedDict


from .message_conversion import get_msg, get_msg_dict, populate_instance, extract_values, FieldTypeMismatchException
from .poolparam import PoolParam
from .topicbase import TopicBase


class SubscriberBackTimeout(Exception):
    pass


class SubscriberBack(TopicBase):
    """
    PublisherBack is the class handling conversion from Python to ROS Publisher
    Requirement : Only one publisherBack per actual ROS Topic.
    Since we connect to an already existing ros topic, our number of connections should never drop under 1
    """

    # We need some kind of instance count here since system state returns only one node instance
    # as publisher for multiple publisher in this process.
    #
    # This is used in ros_interface update for added pubs/subs to determine
    # if we previously started publishing / subscribing to this topic
    # usually the count will be just 1, but more is possible during tests
    #
    # This is also used in ros_interface update for removed pubs/subs to determine
    # if we previously stopped publishing / subscribing to this topic
    # usually the count will be just 1, but more is possible during tests

    pool = PoolParam(rospy.Publisher, "publishers")

    def __init__(self, topic_name, topic_type):

        super(SubscriberBack, self).__init__(topic_name, topic_type)
        # Is 1 a good choice ? # TODO : check which value is best here...

        rospy.loginfo(
            rospy.get_name() + " Pyros.rosinterface : Adding publisher interface {name} {typename}".format(
                name=self.name, typename=self.rostype))

        self.topic = self.pool.acquire(self.name, self.rostype, queue_size=1)
        # CAREFUL ROS publisher doesnt guarantee messages to be delivered
        # stream-like design spec -> loss is acceptable.

        # Make sure we get at least one connection before returning
        start = time.time()
        timeout = 1
        while time.time() - start < timeout and len(self.pool.get_impl_connections(self.name)) < 1:
            # print("publishers connected : {0}".format(self.pool.get_impl_connections(self.name)))
            time.sleep(0.2)

        if self.pool.get_impl_connections(self.name) < 1:
            raise SubscriberBackTimeout

    def cleanup(self):
        """
        Launched when we want to whithhold this interface instance
        :return:
        """
        # TODO : should we do this in del method instead ? to allow reuse until garbage collection actually happens...
        rospy.loginfo(
            rospy.get_name() + " Pyros.rosinterface : Removing publisher interface {name} {typename}".format(
                name=self.name, typename=self.rostype))

        self.pool.release(self.topic)

        super(SubscriberBack, self).cleanup()

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """
        d = super(SubscriberBack, self). asdict()
        d['subscribers'] = self.topic.impl.get_stats_info()
        return d

    def publish(self, msg_content):
        """
        Publishes a message to the topic
        :return the actual message sent if one was sent, None if message couldn't be sent.
        """
        # enforcing correct type to make send / receive symmetric and API less magical
        # Doing message conversion visibly in code before sending into the black magic tunnel sounds like a good idea
        try:
            msg = self.rostype()
            populate_instance(msg_content, msg)
            if isinstance(msg, self.rostype):
                self.topic.publish(msg)  # This should return False if publisher not fully setup yet
                return msg  # because the return spec of rospy's publish is not consistent
        except FieldTypeMismatchException as e:
            rospy.logerr("[{name}] : field type mismatch {e}".format(name=__name__, e=e))
            raise
            # TODO : reraise a topic exception ?
        return None

