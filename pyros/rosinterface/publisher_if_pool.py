from __future__ import absolute_import

import logging
from itertools import ifilter

import rostopic

from .api import rospy_safe as rospy

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

from ..baseinterface import TransientIfPool
from ..baseinterface import DiffTuple

from .topicbase import TopicTuple
from .publisher_if import PublisherBack
from .subscriber_if import SubscriberBack

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None




class RosPublisherIfPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, publishers=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        # CAREFUL publisher interfaces are subscribers
        super(RosPublisherIfPool, self).__init__(publishers, transients_desc="subscribers")

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
        return PublisherBack(topic_name, topic_type, *args, **kwargs)

    def TransientCleaner(self, topic):  # the topic class cleanup implementation
        return topic.cleanup()

    ## bwcompat
    # REQUESTED
    @property
    def publishers_args(self):
        return self.transients_args

    # AVAILABLE
    @property
    def publishers_available(self):
        return self.available

    # INTERFACED
    @property
    def publishers(self):
        return self.transients

    # EXPOSE
    def expose_publishers(self, pub_regex):
        return self.expose_transients_regex(pub_regex)


    def get_publisher_list(self):  # function returning all services available on the system
        return self.get_transients_available

    def publisher_type_resolver(self, service_name):  # function resolving the type of a service
        return self.transient_type_resolver(self, service_name)

    def PublisherMaker(self, topic_name, topic_type):  # the publisher class implementation
        return self.TransientMaker(topic_name, topic_type)

    def PublisherCleaner(self, publisher):  # the publisher class cleanup implementation
        return self.TransientCleaner(publisher)
    ###########

    def reset_state(self, publishers, topic_types):
        """
        Reset internal system state representation.
        expect lists in format similar to masterAPI.
        :param topics
        :param topic_types:
        :return: a Difftuple, suitable to pass directly to
        """

        self.available = dict()
        for t in publishers:  # we assume t[1] is never empty here
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            self.available[ttp.name] = ttp

        return publishers

    def compute_state(self, publishers_dt, topic_types_dt):
        """
        This is called only if there is a cache proxy with a callback, and expects DiffTuple filled up with names or types
        :param topics_dt:
        :return:
        """
        added_pubs = {t[0]: t[1] for t in publishers_dt.added}
        removed_pubs = {t[0]: t[1] for t in publishers_dt.removed}

        for apn, ap in added_pubs.iteritems():
            for rpn, rp in removed_pubs.iteritems():
                if rp in ap:  # remove nodes that are added and removed -> no change seen
                    ap.remove(rp)
                    removed_pubs[rpn].remove(rp)

        for rpn, rp in removed_pubs.iteritems():
            for apn, ap in added_pubs.iteritems():
                if ap in rp:  # remove nodes that are removed and added -> no change seen
                    rp.remove(ap)
                    added_pubs[apn].remove(ap)

        publishers_dt = DiffTuple(
            added=[[k, v] for k, v in added_pubs.iteritems()],
            removed=[[k, v] for k, v in removed_pubs.iteritems()]
        )
        computed_publishers_dt = DiffTuple([], [])
        _logger.debug("topics_dt : {publishers_dt}".format(**locals()))
        for t in publishers_dt.added:
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.added), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            if ttp.name in self.available:
                # if already available, we only update the endpoints list
                self.available[ttp.name].endpoints |= ttp.endpoints
            else:
                self.available[ttp.name] = ttp
                computed_publishers_dt.added.append(t[0])

        for t in publishers_dt.removed:
            tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.removed), [])
            ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
            if ttp.name in self.available:
                self.available[ttp.name].endpoints -= ttp.endpoints
                if not self.available[ttp.name].endpoints:
                    self.available.pop(ttp.name, None)
                    computed_publishers_dt.removed.append(t[0])

        # We still need to return DiffTuples
        return computed_publishers_dt

    def get_pub_interfaces_only_nodes(self):

        pubs_if = SubscriberBack.pool.get_all_interfaces()

        # inverting mapping for nodes ON and OFF separately:
        pubs_if_nodes_on = {}
        pubs_if_nodes_off = {}
        for node, data in pubs_if.iteritems():
            for t, ifon in data.get('publishers', {}).iteritems():  # we need to get the OTHER interfaces (pubs for subs, to prevent detection here)
                if ifon:
                    keys = pubs_if_nodes_on.setdefault(t, set())
                    # we also need to count the number of instance
                    # to avoid dropping Pub/Sub from tests or other annex code.
                    # SPECIAL CASE : normal pyros run should have only one reference to a Pub/sub
                    if PublisherBack.pool.get_impl_ref_count(t) <= 1:
                        keys.add(node)
                else:
                    keys = pubs_if_nodes_off.setdefault(t, set())
                    keys.add(node)

        return pubs_if_nodes_on, pubs_if_nodes_off

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update_delta(self, publishers_dt, topic_types_dt=None):

        # FILTERING OUT TOPICS CREATED BY INTERFACE :

        # First we get all pubs/subs interfaces only nodes
        pubs_if_nodes_on, pubs_if_nodes_off = self.get_pub_interfaces_only_nodes()

        # print(" PUB ADDED DETECTED :")
        # print(publishers_dt.added)

        # Second we filter out ON interface topics from received ADDED topics list
        publishers_dt_added = [
            [t[0], [n for n in t[1]
                    if n not in pubs_if_nodes_on.get(t[0], set())
                    ]
             ]
            for t in publishers_dt.added
            ]

        # filtering out topics with no endpoints
        publishers_dt_added = [[tl[0], tl[1]] for tl in publishers_dt_added if tl[1]]

        # print(" PUB ADDED FILTERED :")
        # print(publishers_dt_added)
        #
        # print(" PUB REMOVED DETECTED :")
        # print(publishers_dt.removed)

        # Second we filter out OFF interface topics from received REMOVED topics list
        publishers_dt_removed = [
            [t[0], [n for n in t[1]
                    #if n not in topics_if_nodes_off.get(t[0], set())
                    # NOT DOABLE CURRENTLY : we would also prevent re interfacing a node that came back up...
                    # Probably better to fix flow between direct update and callback first...
                    # BUT compute_state() should take care of this...
                    ]
             ]
            for t in publishers_dt.removed
            ]

        # # Second we merge in ON interface topics into received REMOVED topics list
        # # This is useful to drop topics interfaces that are satisfying themselves...
        # for t, nodeset in pubs_if_nodes_on.iteritems():
        #     # note manipulating dictionaries will allow us to get rid of this mess
        #     found = False
        #     for td in publishers_dt_removed:
        #         if td[0] == t:
        #             td[1] += nodeset
        #             found = True
        #             break
        #     if not found:
        #         publishers_dt_removed.append([t, list(nodeset)])

        # filtering out topics with no endpoints
        publishers_dt_removed = [[tl[0], tl[1]] for tl in publishers_dt_removed if tl[1]]

        # print(" PUB REMOVED FILTERED :")
        # print(publishers_dt_removed)

        # computing state representation
        publishers_namelist_dt = self.compute_state(DiffTuple(
            added=publishers_dt_added,
            removed=publishers_dt_removed
        ), topic_types_dt or [])

        if publishers_namelist_dt.added or publishers_namelist_dt.removed:
            _logger.debug(
                rospy.get_name() + " Publishers Delta {publishers_namelist_dt}".format(**locals()))

        # TODO : put that in debug log and show based on python logger configuration

        # print("PUBLISHER APPEARED: {publishers_namelist_dt.added}".format(**locals()))
        # print("PUBLISHER GONE : {publishers_namelist_dt.removed}".format(**locals()))

        # update_services wants only names
        dt = self.transient_change_diff(
            transient_appeared=publishers_namelist_dt.added,
            transient_gone=publishers_namelist_dt.removed  # we want only hte name here
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
    def update(self, publishers, topic_types):

        # FILTERING TOPICS CREATED BY INTERFACE :

        # First we get all pubs/subs interfaces only nodes
        pubs_if_nodes_on, pubs_if_nodes_off = self.get_pub_interfaces_only_nodes()

        #print("\n")
        #print(publishers)

        # Second we filter out ALL current and previous interface topics from received topics list
        publishers = [
            [t[0], [n for n in t[1]
                     if (n not in pubs_if_nodes_on.get(t[0], set())  # filter out ON interfaces to avoid detecting interface only topic
                         #n not in topics_if_nodes_off.get(t[0], set())  # filter out OFF interface to avoid re-adding interface that has been recently dropped
                         # NOT DOABLE CURRENTLY : we would also prevent re interfacing a node that came back up...
                         # Probably better to fix flow between direct update and callback first...
                         # BUT reset_state() should take care of this...
                        )
                    ]
            ]
            for t in publishers
        ]

        # TODO : maybe we need to reset the param once the dropped interface have been detected as droped (to be able to start the cycle again)

        #print("\n")
        #print(publishers)

        # filtering out topics with no endpoints
        publishers = [[tl[0], tl[1]] for tl in publishers if tl[1]]

        # First we need to reflect the external system state in internal cache
        self.reset_state(publishers, topic_types)

        #print("\nFULL TOPICS LIST: {topics}".format(**locals()))

        # Second we update our interfaces based on that new system state
        # TODO : pass full topic state here to avoid having to retrieve indirectly
        dt = self.transient_change_detect()

        return dt


TransientIfPool.register(RosPublisherIfPool)

