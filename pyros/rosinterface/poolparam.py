from __future__ import absolute_import

from collections import Counter

from .api import rosnode_safe as rosnode

from .api import rospy_safe as rospy
from .message_conversion import get_msg, get_msg_dict


def get_topic_msg(topic):
    return get_msg(topic.rostype)


def get_topic_msg_dict(topic):
    return get_msg_dict(topic.rostype)


class PoolParam(object):
    """
    This is a pool, supported by the param server in ROS
    It optimizes transient (pub/sub, etc.) creation and reuse, and should play nicely with multiple pyros processes.

    The param name for one interface is :
    - /pyros to gather all pyros instances
    - /<node_name> to gather interfaces by pyros node instances
    - /<if_description> to gather different kind of interfaces (pubs, subs, etc.)
    - /<if_name>

    The param state follows this :
    INTERFACE ON -> param created and set to True -> means currently active
      => if exists and set to true we can ignore the "added interface" from a potential delayed diff feedback
    INTERFACE OFF -> param set to False -> means was active before, currently deactivated
      => if exists and set to false we can ignore the "removed interface" from a potential delayed diff feedback
    param not set means interface was never activated.

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
            # TODO : fix this. This currently can assert because cleanup is not happening when it should (check for shutting_down argument to update()).
            #assert (not rospy.get_param(self.param_namespace + topic_name, False))
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
        Returns the reference counter for this topic interface
        :param name: name of the topic
        :return: reference count of topic interface implementation
        """

        res = 0
        if name in self.topics:
            res = self.topics[name].impl.ref_count
            # TODO : how about topic counter ?
        #print("get_impl_ref_count({name}) => {res}".format(**locals()))
        return res

    def get_impl_connections(self, name):
        """
        Returns the connections for this topic interface
        :param name: name of the topic
        :return: connections of topic interface implementation
        """

        res = []
        if name in self.topics:
            # get usable connection representation as tuple
            res = self.topics[name].impl.get_stats_info()
            # TODO : how about topic counter ?
        #print("get_impl_connections({name}) => {res}".format(**locals()))
        return res

    def get_all_interfaces(self, ros_node=None):
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
        # TODO : use hte param internal interface instead of doing a request here...
        # TODO : or maybe NOT ! The synchronicity of this call is important to know the current state of the interface
        # TODO : getting it from cache suddenly break this mandatory synchronous access...
        pyros_if = rospy.get_param('/pyros', {})
        if_map = {}
        for k, v in pyros_if.iteritems():
            # we need to reconstruct the slashes, lost when storing as params...
            if_map["/" + k] = {
                'uri': rosnode.get_api_uri(rospy.get_master(), "/" + k)[2],
                self.topic_descr: {"/" + tn: tv for tn, tv in flatten_dict(v.get(self.topic_descr, {})).iteritems()}
            }
        return if_map

    def __del__(self):
        rospy.delete_param('/pyros' + rospy.get_name() + '/' + self.topic_descr)

