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

from .topicbase import TopicTuple
from .subscriber_if import SubscriberBack
from .publisher_if import PublisherBack

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None


class RosSubscriberIfPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, subscribers=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        # CAREFUL subscriber interfaces are publishers
        super(RosSubscriberIfPool, self).__init__(subscribers, transients_desc="publishers")

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
        return SubscriberBack(topic_name, topic_type, *args, **kwargs)

    def TransientCleaner(self, subscriber):  # the topic class cleanup implementation
        return subscriber.cleanup()

    ## bwcompat
    # REQUESTED
    @property
    def subscribers_args(self):
        return self.transients_args

    # AVAILABLE
    @property
    def subscribers_available(self):
        return self.available

    # INTERFACED
    @property
    def subscribers(self):
        return self.transients

    # EXPOSE
    def expose_subscribers(self, tpc_regex):
        return self.expose_transients_regex(tpc_regex)


    def get_subscriber_list(self):  # function returning all services available on the system
        return self.get_transients_available

    def subscriber_type_resolver(self, service_name):  # function resolving the type of a service
        return self.transient_type_resolver(self, service_name)


    def SubscriberMaker(self, service_name, service_type):  # the service class implementation
        return self.TransientMaker(service_name, service_type)

    def SubscriberCleaner(self, service):  # the service class cleanup implementation
        return self.TransientCleaner(service)
    ###########

    def reset_state(self, subscribers, topic_types):
        """
        Reset internal system state representation.
        expect lists in format similar to masterAPI.
        :param subscribers
        :param topic_types:
        :return: a Difftuple, suitable to pass directly to
        """

        self.available = dict()
        for s in subscribers:  # we assume t[1] is never empty here
            tt = next(ifilter(lambda ltt: s[0] == ltt[0], topic_types), [])
            ttp = TopicTuple(name=s[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(s[1]))
            self.available[ttp.name] = ttp

        return subscribers

    def compute_state(self, subscribers_dt, topic_types_dt):
        """
        This is called only if there is a cache proxy with a callback, and expects DiffTuple filled up with names or types
        :param topics_dt:
        :return:
        """
        added_subs = {t[0]: t[1] for t in subscribers_dt.added}
        removed_subs = {t[0]: t[1] for t in subscribers_dt.removed}

        for apn, ap in added_subs.iteritems():
            for rpn, rp in removed_subs.iteritems():
                if rp in ap:  # remove nodes that are added and removed -> no change seen
                    ap.remove(rp)
                    removed_subs[rpn].remove(rp)

        for rpn, rp in removed_subs.iteritems():
            for apn, ap in added_subs.iteritems():
                if ap in rp:  # remove nodes that are removed and added -> no change seen
                    rp.remove(ap)
                    added_subs[apn].remove(ap)

        subscribers_dt = DiffTuple(
            added=[[k, v] for k, v in added_subs.iteritems()],
            removed=[[k, v] for k, v in removed_subs.iteritems()]
        )
        computed_subscribers_dt = DiffTuple([], [])
        _logger.debug("removed_subs_dt : {subscribers_dt}".format(**locals()))
        for t in subscribers_dt.added:
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.added), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            if ttp.name in self.available:
                # if already available, we only update the endpoints list
                self.available[ttp.name].endpoints |= ttp.endpoints
                # no change here, no need to add that topic to the computed diff.
            else:
                self.available[ttp.name] = ttp
                computed_subscribers_dt.added.append(t[0])

        for t in subscribers_dt.removed:
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.removed), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            if ttp.name in self.available:
                self.available[ttp.name].endpoints -= ttp.endpoints
                if not self.available[ttp.name].endpoints:
                    self.available.pop(ttp.name, None)
                    computed_subscribers_dt.removed.append(t[0])

        # We still need to return DiffTuples
        return computed_subscribers_dt

    def get_sub_interfaces_only_nodes(self):

        subs_if = PublisherBack.pool.get_all_interfaces()

        subs_if_nodes_on = {}
        subs_if_nodes_off = {}
        for node, data in subs_if.iteritems():
            for t, ifon in data.get('subscribers', {}).iteritems():  # we need to get the OTHER interfaces (subs for pubs, to prevent detection here)
                if ifon:
                    keys = subs_if_nodes_on.setdefault(t, set())
                    # we also need to count the number of instance
                    # to avoid dropping Pub/Sub from tests or other annex code.
                    # SPECIAL CASE : normal pyros run should have only one reference to a Pub/sub
                    if SubscriberBack.pool.get_impl_ref_count(t) <= 1:
                        keys.add(node)
                else:
                    keys = subs_if_nodes_off.setdefault(t, set())
                    keys.add(node)

        return subs_if_nodes_on, subs_if_nodes_off

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update_delta(self, subscribers_dt, topic_types_dt=None):

        # FILTERING OUT TOPICS CREATED BY INTERFACE :

        # First we get all pubs/subs interfaces only nodes
        subs_if_nodes_on, subs_if_nodes_off = self.get_sub_interfaces_only_nodes()

        # print(" SUB ADDED DETECTED :")
        # print(subscribers_dt.added)

        # Second we filter out ON interface topics from received ADDED topics list
        subscribers_dt_added = [
            [t[0], [n for n in t[1]
                    if n not in subs_if_nodes_on.get(t[0], set())
                    ]
             ]
            for t in subscribers_dt.added
            ]

        # filtering out topics with no endpoints
        subscribers_dt_added = [[tl[0], tl[1]] for tl in subscribers_dt_added if tl[1]]

        # print(" SUB ADDED FILTERED :")
        # print(subscribers_dt_added)
        #
        # print(" SUB REMOVED DETECTED :")
        # print(subscribers_dt.removed)

        # Second we filter out OFF interface topics from received REMOVED topics list
        subscribers_dt_removed = [
            [t[0], [n for n in t[1]
                    #if n not in topics_if_nodes_off.get(t[0], set())
                    # NOT DOABLE CURRENTLY : we would also prevent re interfacing a node that came back up...
                    # Probably better to fix flow between direct update and callback first...
                    # BUT compute_state() should take care of this...
                    ]
             ]
            for t in subscribers_dt.removed
            ]

        # # Second we merge in ON interface topics into received REMOVED topics list
        # # This is useful to drop topics interfaces that are satisfying themselves...
        # for t, nodeset in subs_if_nodes_on.iteritems():
        #     # note manipulating dictionaries will allow us to get rid of this mess
        #     found = False
        #     for td in subscribers_dt_removed:
        #         if td[0] == t:
        #             td[1] += nodeset
        #             found = True
        #             break
        #     if not found:
        #         subscribers_dt_removed.append([t, list(nodeset)])

        # filtering out topics with no endpoints
        subscribers_dt_removed = [[tl[0], tl[1]] for tl in subscribers_dt_removed if tl[1]]

        # print(" SUB REMOVED FILTERED :")
        # print(subscribers_dt_removed)

        # computing state representation
        subscribers_namelist_dt = self.compute_state(DiffTuple(
            added=subscribers_dt_added,
            removed=subscribers_dt_removed
        ), topic_types_dt or [])

        if subscribers_namelist_dt.added or subscribers_namelist_dt.removed:
            _logger.debug(
                rospy.get_name() + " Subscribers Delta {subscribers_namelist_dt}".format(**locals()))

        # TODO : put that in debug log and show based on python logger configuration

        # print("SUBSCRIBER APPEARED: {subscribers_namelist_dt.added}".format(**locals()))
        # print("SUBSCRIBER GONE : {subscribers_namelist_dt.removed}".format(**locals()))

        # update_services wants only names
        dt = self.transient_change_diff(
            transient_appeared=subscribers_namelist_dt.added,
            transient_gone=subscribers_namelist_dt.removed  # we want only hte name here
            # add_names=regexes_match_sublist(self.transients_args, [s[0] for s in topics_dt.added]),
            # remove_names=[s[0] for s in topics_dt.removed if s[0] not in self.get_transients_available()]
        )

        if dt.added or dt.removed:
            _logger.debug(
                rospy.get_name() + " Update Delta {dt}".format(**locals()))
            # print(" UPDATE DELTA:")
            # print(dt)
        return dt

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update(self, subscribers, topic_types):

        # FILTERING TOPICS CREATED BY INTERFACE :

        # First we get all pubs/subs interfaces only nodes
        subs_if_nodes_on, subs_if_nodes_off = self.get_sub_interfaces_only_nodes()


        # Second we filter out ALL current and previous interface topics from received topics list
        subscribers = [
            [t[0], [n for n in t[1]
                     if (n not in subs_if_nodes_on.get(t[0], set())  # filter out ON interfaces to avoid detecting interface only topic
                         #n not in topics_if_nodes_off.get(t[0], set())  # filter out OFF interface to avoid re-adding interface that has been recently dropped
                         # NOT DOABLE CURRENTLY : we would also prevent re interfacing a node that came back up...
                         # Probably better to fix flow between direct update and callback first...
                          # BUT reset_state() should take care of this...
                        )
                    ]
            ]
            for t in subscribers
        ]

        # TODO : maybe we need to reset the param once the dropped interface have been detected as droped (to be able to start the cycle again)

        #print("\n")
        #print(topics)

        # filtering out topics with no endpoints
        subscribers = [[tl[0], tl[1]] for tl in subscribers if tl[1]]

        # First we need to reflect the external system state in internal cache
        self.reset_state(subscribers, topic_types)

        #print("\nFULL TOPICS LIST: {topics}".format(**locals()))

        # Second we update our interfaces based on that new system state
        # TODO : pass full topic state here to avoid having to retrieve indirectly
        dt = self.transient_change_detect()

        return dt


TransientIfPool.register(RosSubscriberIfPool)

