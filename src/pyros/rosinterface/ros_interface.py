from __future__ import absolute_import

import rospy
import rosservice, rostopic, rosparam

import re
import ast
import socket

from pyros.baseinterface import BaseInterface

from .service import ServiceBack
from .topic import TopicBack
from .param import ParamBack


class RosInterface(BaseInterface):
    """
    RosInterface.
    """
    def __init__(self, services=None, topics=None, params=None):
        # Current mock implementation of services, topics and params
        super(RosInterface, self).__init__(services or [], topics or [], params or [])
        self.services_available = set()
        self.topics_available = set()
        self.params_available = set()
        # connecting to the master via proxy object
        self._master = rospy.get_master()

        # Setting our list of interfaced topic right when we start
        rospy.set_param('~' + TopicBack.IF_TOPIC_PARAM, [])



    # ros functions that should connect with the ros system we want to interface with
    # SERVICES
    def get_svc_list(self):  # function returning all services available on the system
        return self.services_available

    def service_type_resolver(self, service_name):  # function resolving the type of a service
        try:
            resolved_service_name = rospy.resolve_name(service_name)  # required or not ?
            service_type = rosservice.get_service_type(resolved_service_name)  # maybe better to store and retrieve from update instead?
        except rosservice.ROSServiceIOException:  # exception can occur -> just reraise
           raise
        return service_type

    def ServiceMaker(self, service_name, service_type):  # the service class implementation
        return ServiceBack(service_name, service_type)

    def ServiceCleaner(self, service):  # the service class cleanup implementation
        return service.cleanup()

    # TOPICS
    def get_topic_list(self):  # function returning all topics available on the system
        return self.topics_available

    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        try:
            resolved_topic_name = rospy.resolve_name(topic_name)
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
        except rosservice.ROSServiceIOException, e:  # exception can occur -> just reraise
            raise
        return topic_type

    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return TopicBack(topic_name, topic_type, *args, **kwargs)

    def TopicCleaner(self, topic):  # the topic class implementation
        return topic.cleanup()
    # PARAMS
    def get_param_list(self):  # function returning all params available on the system
        return self.params_available

    def param_type_resolver(self, param_name):  # function resolving the type of a param
        return self.params_available_type.get(param_name, None)

    def ParamMaker(self, param_name, param_type):  # the param class implementation
        return ParamBack(param_name, param_type)

    def ParamCleaner(self, param):  # the param class implementation
        return param.cleanup()

    def reinit(self, services=None, topics=None, params=None):
        # Note : None means no change ( different from []
        super(RosInterface, self).reinit(services, topics, params)

    def update(self):
        """
        Redefining update method in child class to gather all information in one master call per update.
        :return Return the difference tuple of services/topics/params exposed/withhold (NOT the detected changes we should not care about).
                Note that the name must match a regex previously set by the expose call
        """
        # TODO : improve this by using another node that caches master state and provide safe access
        try:
            _, _, system_state = self._master.getSystemState()
            publishers, subscribers, services = system_state

            params = rospy.get_param_names()

            # getting the list of interfaced topics from well known node param
            if_topics = {}
            for par in params:
                if par.endswith(TopicBack.IF_TOPIC_PARAM):
                    # extract process name from param name ( removing extra slash )
                    pname = par[:- len('/' + TopicBack.IF_TOPIC_PARAM)]
                    if_topics[pname] = rospy.get_param(par, [])

            # TODO : improvement : get the types here already ( no need to recall master everytime to get it )
            # Current status : getting all topic types is another master request and might not be always needed
            #  -> dont call again the master from here.

            # Examination of topics :
            # We keep publishers that is provided by something else ( not our exposed topic pub if present )
            # OR if we have locally multiple pubs / subs.
            filtered_publishers = []
            for p in publishers:
                # keeping only nodes that are not pyros interface for this topic
                nonif_pub_providers = [pp for pp in p[1] if (p[0] not in if_topics.get(pp, []) or TopicBack.pub_instance_count.get(p[0], 0) > 1)]
                # also keeping interface nodes that have more than one interface (useful for tests and nodelets, etc. )
                if nonif_pub_providers:
                    filtered_publishers.append([p[0], nonif_pub_providers])

            # We keep subscribers that are provided by something else ( not our exposed topic sub if present )
            # OR if we have locally multiple pubs / subs.
            filtered_subscribers = []
            for s in subscribers:
                # keeping only nodes that are not pyros interface for this topic
                nonif_sub_providers = [sp for sp in s[1] if (s[0] not in if_topics.get(sp, []) or TopicBack.sub_instance_count.get(s[0], 0) > 1)]
                # also keeping interface nodes that have more than one interface (useful for tests and nodelets, etc. )
                if nonif_sub_providers:
                    filtered_subscribers.append([s[0], nonif_sub_providers])

            # We merge both pubs and subs, so that only one pub or one sub which is not ours is enough to keep the topic
            self.topics_available = set([t[0] for t in (filtered_publishers + filtered_subscribers)])
            self.services_available = set([s[0] for s in services])

            self.params_available = set(params)

        except socket.error:
            rospy.logerr("[] couldn't get system state from the master ")

        return super(RosInterface, self).update()


BaseInterface.register(RosInterface)


