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
        :return: a Difftuple, suitable to pass directly to
        """

        self.available = dict()
        for t in topics:  # we assume t[1] is never empty here
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            self.available[ttp.name] = ttp

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
        _logger.debug("topics_dt : {topics_dt}".format(**locals()))
        for t in topics_dt.added:
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.added), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            if ttp.name in self.available:
                # if already available, we only update the endpoints list
                self.available[ttp.name].endpoints |= ttp.endpoints
            else:
                self.available[ttp.name] = ttp

        for t in topics_dt.removed:
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.removed), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            if ttp.name in self.available:
                self.available[ttp.name].endpoints -= ttp.endpoints
                if not self.available[ttp.name].endpoints:
                    self.available.pop(ttp.name, None)

        # We still need to return DiffTuples
        return topics_dt

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update_delta(self, topics_dt, topic_types_dt=None):

        # FILTERING TOPICS CREATED BY INTERFACE :

        # First we get all pubs/subs interfaces
        subs_if = TopicBack.get_all_sub_interfaces()
        pubs_if = TopicBack.get_all_pub_interfaces()

        # inverting mapping for nodes ON and OFF separately:
        pubs_if_nodes_on = {}
        pubs_if_nodes_off = {}
        for node, data in pubs_if.iteritems():
            for t, ifon in data.get('publishers', {}).iteritems():
                if ifon:
                    keys = pubs_if_nodes_on.setdefault(t, set())
                    keys.add(node)
                else:
                    keys = pubs_if_nodes_off.setdefault(t, set())
                    keys.add(node)


        subs_if_nodes_on = {}
        subs_if_nodes_off = {}
        for node, data in subs_if.iteritems():
            for t, ifon in data.get('subscribers', {}).iteritems():
                if ifon:
                    keys = subs_if_nodes_on.setdefault(t, set())
                    keys.add(node)
                else:
                    keys = subs_if_nodes_off.setdefault(t, set())
                    keys.add(node)

        print("\n")
        print(topics_dt.added)

        # Second we filter out ON interface topics from received ADDED topics list
        topics_dt_added = [
            [t[0], [n for n in t[1]
                    if n not in pubs_if_nodes_on.get(t[0], set()) and n not in subs_if_nodes_on.get(t[0], set())
                    ]
             ]
            for t in topics_dt.added
            ]

        # filtering out topics with no endpoints
        topics_dt_added = [[tl[0], tl[1]] for tl in topics_dt_added if tl[1]]

        print("\n")
        print(topics_dt_added)

        print("\n")
        print(topics_dt.removed)

        # Second we filter out OFF interface topics from received REMOVED topics list
        topics_dt_removed = [
            [t[0], [n for n in t[1]
                    if n not in pubs_if_nodes_off.get(t[0], set()) and n not in subs_if_nodes_off.get(t[0], set())
                    ]
             ]
            for t in topics_dt.removed
            ]

        # filtering out topics with no endpoints
        topics_dt_removed = [[tl[0], tl[1]] for tl in topics_dt_removed if tl[1]]

        print("\n")
        print(topics_dt_removed)


        # computing state representation
        topics_dt = self.compute_state(DiffTuple(
            added=topics_dt_added,
            removed=topics_dt_removed
        ), topic_types_dt or [])

        if topics_dt.added or topics_dt.removed:
            _logger.debug(
                rospy.get_name() + " Topics Delta {topics_dt}".format(**locals()))

        # TODO : put that in debug log and show based on python logger configuration
        # print("Pyros ROS interface UPDATE")
        # print("Topics ADDED: {0}".format([s[0] for s in topics_dt.added]))
        # print("Topics GONE: {0}".format([s[0] for s in topics_dt.removed]))

        print("\nTOPIC APPEARED: {topics_dt.added}".format(**locals()))
        print("TOPIC GONE : {topics_dt.removed}".format(**locals()))

        # update_services wants only names
        dt = self.transient_change_diff(
            transient_appeared=[t[0] for t in topics_dt_added],
            transient_gone=[t[0] for t in topics_dt_removed]  # we want only hte name here
            # add_names=regexes_match_sublist(self.transients_args, [s[0] for s in topics_dt.added]),
            # remove_names=[s[0] for s in topics_dt.removed if s[0] not in self.get_transients_available()]
        )

        if dt.added or dt.removed:
            _logger.debug(
                rospy.get_name() + " Update Delta {dt}".format(**locals()))
        return dt

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update(self, topics, topic_types):

        # FILTERING TOPICS CREATED BY INTERFACE :

        # First we get all pubs/subs interfaces
        subs_if = TopicBack.get_all_sub_interfaces()
        pubs_if = TopicBack.get_all_pub_interfaces()

        # inverting mapping :
        pubs_if_nodes_on = {}
        pubs_if_nodes_off = {}
        for node, data in pubs_if.iteritems():
            for t, ifon in data.get('publishers', {}).iteritems():
                if ifon:
                    keys = pubs_if_nodes_on.setdefault(t, set())
                    keys.add(node)
                else:
                    keys = pubs_if_nodes_off.setdefault(t, set())
                    keys.add(node)

        subs_if_nodes_on = {}
        subs_if_nodes_off = {}
        for node, data in subs_if.iteritems():
            for t, ifon in data.get('subscribers', {}).iteritems():
                if ifon:
                    keys = subs_if_nodes_on.setdefault(t, set())
                    keys.add(node)
                else:
                    keys = subs_if_nodes_off.setdefault(t, set())
                    keys.add(node)

        #print("\n")
        #print(topics)

        # Second we filter out ALL current and previous interface topics from received topics list
        topics = [
            [t[0], [n for n in t[1]
                     if (n not in pubs_if_nodes_on.get(t[0], set()) and  # filter out ON interfaces to avoid detecting interface only topic
                         n not in subs_if_nodes_on.get(t[0], set()) and
                         n not in pubs_if_nodes_off.get(t[0], set()) and  # filter out OFF interface to avoid detecting interface that has been recently dropped
                         n not in subs_if_nodes_on.get(t[0], set()))
                     ]
            ]
            for t in topics
        ]

        #print("\n")
        #print(topics)

        # filtering out topics with no endpoints
        topics = [[tl[0], tl[1]] for tl in topics if tl[1]]

        # First we need to reflect the external system state in internal cache
        self.reset_state(topics, topic_types)

        print("\nFULL TOPICS LIST: {topics}".format(**locals()))

        # Second we update our interfaces based on that new system state
        # TODO : pass full topic state here to avoid having to retrieve indirectly
        dt = self.transient_change_detect()

        return dt


TransientIfPool.register(RosTopicIfPool)

