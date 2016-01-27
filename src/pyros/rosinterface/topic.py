from __future__ import absolute_import

import time

import roslib
import rospy

from importlib import import_module
from collections import deque, OrderedDict


from .message_conversion import get_msg, get_msg_dict


def get_topic_msg(topic):
    return get_msg(topic.rostype)


def get_topic_msg_dict(topic):
    return get_msg_dict(topic.rostype)


class TopicBackTimeout(Exception):
    pass


class TopicBack(object):
    """
    TopicBack is the class handling conversion from Python to ROS Topic
    Requirement : Only one topicBack per actual ROS Topic.
    Since we connect to an already existing ros topic, our number of connections should never drop under 1
    """

    # We need some kind of instance count here since system state returns only one node instance
    # as publisher for multiple publisher in this process.
    # This is used in ros_interface update to determine
    # if we are the only last ones publishing / subscribing to this topic ( usually the count will be just 1 )

    # TO have this working properly with multiple instance, we need a central place to keep that
    pub_instance_count = {}
    sub_instance_count = {}

    # a solution that works multiprocess
    IF_TOPIC_PARAM = 'pyros_if_topics'

    # TODO: maybe contextmanager to make sure unregister always happens if we create it ?
    # Should be fine since hte interface is in loop...
    # this might help keep track of the interface created pub/sub, and help keep things tidy...

    @staticmethod
    def _create_pub(name, rostype, *args, **kwargs):
        """
        Creating a publisherm and adding it to the pub instance count.
        Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
        :return: the ros publisher
        """
        # counting publisher instance per topic name
        if name in TopicBack.pub_instance_count.keys():
            TopicBack.pub_instance_count[name] += 1
        else:
            TopicBack.pub_instance_count[name] = 1

        return rospy.Publisher(name, rostype, *args, **kwargs)

    @staticmethod
    def _remove_pub(pub):
        """
        Removing a publisher and substracting it from the pub instance count.
        :return: None
        """
        # counting publisher instance per topic name
        TopicBack.pub_instance_count[pub.name] -= 1

        # Be aware of https://github.com/ros/ros_comm/issues/111
        return pub.unregister()

    @staticmethod
    def _create_sub(name, rostype, topic_callback, *args, **kwargs):
        """
        Creating a subscriber and adding it to the pub instance count.
        Static and Functional style so we can call it from anywhere (tests).
        Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
        :return: the subscriber
        """
        # counting subscriber instance per topic name
        if name in TopicBack.sub_instance_count.keys():
            TopicBack.sub_instance_count[name] += 1
        else:
            TopicBack.sub_instance_count[name] = 1

        return rospy.Subscriber(name, rostype, topic_callback, *args, **kwargs)

    @staticmethod
    def _remove_sub(sub):
        """
        Creating a publisher and adding it to the pub instance count.
        Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
        :return: None
        """
        # counting publisher instance per topic name
        TopicBack.sub_instance_count[sub.name] -= 1

        # Be aware of https://github.com/ros/ros_comm/issues/111
        return sub.unregister()

    def __init__(self, topic_name, topic_type, queue_size=1, start_timeout=2):

        self.name = topic_name

        # TODO : think about if we enforce fullname before reaching this deep in the code
        # Any benefit with relative names ?

        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        topic_type_module, topic_type_name = tuple(topic_type.split('/'))
        roslib.load_manifest(topic_type_module)
        msg_module = import_module(topic_type_module + '.msg')

        self.rostype_name = topic_type
        self.rostype = getattr(msg_module, topic_type_name)

        self.msgtype = get_topic_msg_dict(self)
        self.msg = deque([], queue_size)

        self.pub = None

        self.pub = self._create_pub(self.fullname, self.rostype, queue_size=1)
        # CAREFUL ROS publisher doesnt guarantee messages to be delivered
        # stream-like design spec -> loss is acceptable.
        self.sub = self._create_sub(self.fullname, self.rostype, self.topic_callback)

        # Advertising ROS system wide, which topic are interfaced with this process
        # TODO : make this thread safe
        if_topics = rospy.get_param('~' + TopicBack.IF_TOPIC_PARAM, [])
        rospy.set_param('~' + TopicBack.IF_TOPIC_PARAM, if_topics + [self.fullname])

        # Here making sure the publisher / subscriber pair is actually connected
        # before returning to ensure RAII
        start = time.time()
        timeout = start_timeout
        while time.time() - start < timeout and (
            self.pub.get_num_connections() < 1 or
            self.sub.get_num_connections() < 1
        ):
            rospy.rostime.wallsleep(1)
        if start - time.time() > timeout:
            raise TopicBackTimeout()

        self.empty_cb = None

    def cleanup(self):
        """
        Launched when we want to whithhold this interface instance
        :return:
        """
        # Removing the ROS system wide advert about which topic are interfaced with this process
        # TODO : lock this for concurrent access
        if_topics = rospy.get_param('~' + TopicBack.IF_TOPIC_PARAM, [])
        if_topics.remove(self.fullname)
        rospy.set_param('~' + TopicBack.IF_TOPIC_PARAM, if_topics)

        # cleanup pub and sub, so we can go through another create / remove cycle properly
        self._remove_pub(self.pub)
        self._remove_sub(self.sub)

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """
        return OrderedDict({
            'name': self.name,
            'fullname': self.fullname,
            'msgtype': self.msgtype,
            'rostype_name': self.rostype_name,
        })

    def publish(self, msg):
        # enforcing correct type to make send / receive symmetric and API less magical
        # Doing message conversion visibly in code before sending into the black magic tunnel sounds like a good idea
        if isinstance(msg, self.rostype):
            self.pub.publish(msg)  # This should return False if publisher not fully setup yet
            return True  # because the return spec of rospy's publish is not consistent
        return False

    def get(self, num=0, consume=False):
        if not self.msg:
            return None

        res = None
        #TODO : implement returning multiple messages ( paging/offset like for long REST requests )
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

    def topic_callback(self, msg):
        self.msg.appendleft(msg)

