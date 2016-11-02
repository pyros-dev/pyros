from __future__ import absolute_import

import os
from collections import namedtuple, MutableMapping
from copy import deepcopy, copy
from itertools import ifilter
import logging

import pyros_utils
import rospy
import rosservice, rostopic, rosparam

import re
import ast
import socket
import threading
import Queue

import time

from ..baseinterface.regex_tools import regexes_match_sublist

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

from ..baseinterface import TransientIfPool
from ..baseinterface import DiffTuple

from .topic import TopicBack, TopicTuple

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None



class RosTopicIfPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, topics=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        super(RosTopicIfPool, self).__init__(topics, transients_desc="topics")

    def get_transients_available(self):  # function returning all services available on the system
        return self.available

    def transient_type_resolver(self, topic_name):  # function resolving the type of a service
        tpc = self.available.get(topic_name)
        if tpc:
            if tpc.type is None:  # if the type is unknown, lets discover it
                try:
                    resolved_topic_name = rospy.resolve_name(topic_name)
                    tpc.type, _, _ = rostopic.get_topic_type(resolved_topic_name)
                except rostopic.ROSTopicIOException:  # exception can occur -> just reraise
                   raise
            return tpc.type  # return the first we find. enough.
        else:
            rospy.logerr("ERROR while resolving {topic_name}. Topic not known as available. Ignoring".format(**locals()))
            return None

    def TransientMaker(self, topic_name, topic_type, *args, **kwargs):  # the service class implementation
        return TopicBack(topic_name, topic_type, *args, **kwargs)

    def TransientCleaner(self, topic):  # the topic class cleanup implementation
        return topic.cleanup()

    ## bwcompat
    def get_topic_list(self):  # function returning all services available on the system
        return self.get_transients_available

    def topic_type_resolver(self, service_name):  # function resolving the type of a service
        return self.transient_type_resolver(self, service_name)


    def TopicMaker(self, service_name, service_type):  # the service class implementation
        return self.TransientMaker(service_name, service_type)

    def TopicCleaner(self, service):  # the service class cleanup implementation
        return self.TransientCleaner(service)
    ###########

    def reset_state(self, topics, topic_types):
        """
        Reset internal system state representation.
        expect lists in format similar to masterAPI.
        :param topics
        :param topic_types:
        :return:
        """
        with self.available_lock:
            self.available = dict()
            for t in topics:  # we assume t[1] is never empty here
                tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types), [])
                ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
                self.available[ttp.name] = ttp

        print("AVAILABLE TOPICS RESET => {topics}".format(**locals()))
        return topics

    def compute_state(self, topics_dt, topic_types_dt):
        """
        This is called only if there is a cache proxy with a callback, and expects DiffTuple filled up with names or types
        :param topics_dt:
        :return:
        """
        added_topics = {t[0]: t[1] for t in topics_dt.added}
        removed_topics = {t[0]: t[1] for t in topics_dt.removed}

        for apn, ap in added_topics.iteritems():
            for rpn, rp in removed_topics.iteritems():
                if rp in ap:  # remove nodes that are added and removed -> no change seen
                    ap.remove(rp)
                    removed_topics[rpn].remove(rp)

        for rpn, rp in removed_topics.iteritems():
            for apn, ap in added_topics.iteritems():
                if ap in rp:  # remove nodes that are removed and added -> no change seen
                    rp.remove(ap)
                    added_topics[apn].remove(ap)

        # # We merge both pubs and subs, so that only one pub or one sub which is not ours is enough to keep the topic
        # # Need to be careful if pub and sub are added/removed at same time : only one topic added/removed
        # added_topics = {pub[0]: pub[1] for pub in filtered_added_publishers}
        # removed_topics = {pub[0]: pub[1] for pub in filtered_removed_publishers}
        #
        # for t in filtered_added_subscribers:
        #     added_topics[t[0]] = added_topics.get(t[0], []) + t[1]
        # for t in filtered_removed_subscribers:
        #     removed_topics[t[0]] = removed_topics.get(t[0], []) + t[1]

        # TODO : improve here to make sure of nodes unicity after this collapsing step

        topics_dt = DiffTuple(
            added=[[k, v] for k, v in added_topics.iteritems()],
            removed=[[k, v] for k, v in removed_topics.iteritems()]
        )
        self._debug_logger.debug("topics_dt : {topics_dt}".format(**locals()))
        with self.available_lock:
            for t in topics_dt.added:
                tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.added), [])
                ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
                if ttp.name in self.topics_available:
                    # if already available, we only update the endpoints list
                    self.available[ttp.name].endpoints |= ttp.endpoints
                else:
                    self.available[ttp.name] = ttp

            for t in topics_dt.removed:
                tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.removed), [])
                ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
                if ttp.name in self.topics_available:
                    self.available[ttp.name].endpoints -= ttp.endpoints
                    if not self.available[ttp.name].endpoints:
                        self.available.pop(ttp.name, None)

        # We still need to return DiffTuples
        return topics_dt






    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update_delta(self, services_dt, service_types_dt=None):
        services_dt = self.compute_state(services_dt, service_types_dt or [])

        if services_dt.added or services_dt.removed:
            self._debug_logger.debug(
                rospy.get_name() + " Pyros.rosinterface.service_if_pool : Services Delta {services_dt}".format(**locals()))

        # TODO : put that in debug log and show based on python logger configuration
        # print("Pyros ROS interface UPDATE")
        # print("Srvs ADDED: {0}".format([s[0] for s in services_dt.added]))
        # print("Srvs GONE: {0}".format([s[0] for s in services_dt.removed]))

        # update_services wants only names
        dt = self.update_transients(
            add_names=regexes_match_sublist(self.transients_args, [s[0] for s in services_dt.added]),
            remove_names=[s[0] for s in services_dt.removed if s[0] not in self.get_transients_available()]
            )

        if dt.added or dt.removed:
            self._debug_logger.debug(
                rospy.get_name() + " Pyros.rosinterface.service_if_pool : Update Delta {dt}".format(**locals()))
        return dt


    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update(self, topics, topic_types):
        # First we need to reflect the external system state in internal cache
        self.reset_state(topics, topic_types)

        # Second we update our interfaces based on that new system state
        # TODO : pass full topic state here to avoid having to retrieve indirectly
        dt = self.transient_change_detect()

        return dt


TransientIfPool.register(RosTopicIfPool)

