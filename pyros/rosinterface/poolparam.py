from __future__ import absolute_import

import time

import roslib
import rospy

from importlib import import_module
from collections import Counter


from .message_conversion import get_msg, get_msg_dict, populate_instance, extract_values, FieldTypeMismatchException


def get_topic_msg(topic):
    return get_msg(topic.rostype)


def get_topic_msg_dict(topic):
    return get_msg_dict(topic.rostype)


class PoolParam(object):
    """
    This is a pool, supported by the param server in ROS
    It optimizes transient (pub/sub, etc.) creation and reuse, and should play nicely with multiple pyros processes.
    """

    # need to wait for ROS to start before we can get a name...
    @property
    def param_namespace(self):
        return '/pyros' + rospy.get_name() + '/' + self.topic_descr

    def __init__(self, topic_class, topic_descr):
        # :topic_class rospy.Publisher or rospy.Subscriber
        self.topic_class = topic_class
        # :topic_descr "publishers" or "subscribers"
        self.topic_descr = topic_descr
        # ROS didnt start yet we cant write a param.
        # rospy.set_param(self.param_namespace, {})

        # setting up the pool for this instance
        self.topics = {}
        self.topics_count = Counter()

    def acquire(self, topic_name, topic_type, *args, **kwargs):
        """
        Creating a publisher (if needed) and adding it to the pub instance count.
        Otherwise will reuse an existing publisher for the same topic.
        :return: the ros publisher
        """
        # NOTE : the node must be initialized before reaching this
        topic_name = rospy.resolve_name(topic_name)
        if topic_name not in self.topics:
            # Asserting this topic interface is also not registered on ROS param server
            assert (not rospy.get_param(self.param_namespace + topic_name, False))
            # build a new instance only if needed
            self.topics[topic_name] = self.topic_class(topic_name, topic_type, *args, **kwargs)

            # Advertising ROS system wide, which topic are interfaced with this process
            #  We need this to be atomic to avoid race conditions
            rospy.set_param(self.param_namespace + topic_name, True)

        # assert topic type (data_class) didn't change in the meantime (ROS doesnt support it anyway)
        assert (topic_type == self.topics[topic_name].data_class)
        # We count artificial instances here (rospy uses same socket internally).
        # this helps using only one publisher in the interface.
        self.topics_count[topic_name] += 1
        # but that data has meaning only in this process. other processes will see only one connection.
        return self.topics[topic_name]

    def release(self, tpc):
        """
        Removing a topic and substracting it from the list.
        :return: None
        """
        if tpc.name in self.topics_count:
            self.topics_count[tpc.name] -= 1
            if self.topics_count[tpc.name] == 0:
                # Advertising ROS system wide, which topic are interfaced with this process
                #  We need this to be atomic to avoid race conditions
                rospy.set_param(self.param_namespace + tpc.name, False)

                self.topics.pop(tpc.name)
                # TODO: keep it around until GC ??
                # Be aware of https://github.com/ros/ros_comm/issues/111
                tpc.unregister()

    def get_impl_ref_count(self, name):
        """
        Check whether the topic interface is the last pub/sub instance present
        returns False if not present
        :param name: name of the topic
        :param ros_num_connections: num_connections for this sub, as reported by rospy
        :return: True/False
        """

        res = 0
        if name in self.topics:
            res = self.topics[name].impl.ref_count
            # TODO : how about topic counter ?
        print("get_impl_ref_count({name}) => {res}".format(**locals()))
        return res

    def get_all_interfaces(self):
        # Parameters will interpret / as a sub mapping collection
        # We need to flatten it to retrieve topic names...
        def flatten_dict(d):
            def expand(key, value):
                if isinstance(value, dict):
                    return [(key + '/' + k, v) for k, v in flatten_dict(value).items()]
                else:
                    return [(key, value)]

            items = [item for k, v in d.items() for item in expand(k, v)]

            return dict(items)

        # Inspect params to find who also interface this publisher
        pyros_if = rospy.get_param('/pyros', {})
        if_map = {}
        for k, v in pyros_if.iteritems():
            # we need to reconstruct the slashes, lost when storing as params...
            if_map["/" + k] = {"/" + tn: tv for tn, tv in flatten_dict(v.get(self.topic_descr, {})).iteritems()}
        return if_map

    def __del__(self):
        rospy.delete_param('/pyros' + rospy.get_name() + '/' + self.topic_descr)

