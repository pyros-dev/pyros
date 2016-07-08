from __future__ import absolute_import

import os
from collections import namedtuple, MutableMapping
from copy import deepcopy, copy
from itertools import ifilter
import logging

import rospy
import rosservice, rostopic, rosparam

import re
import ast
import socket
import threading
import Queue

import time
from ..baseinterface import BaseInterface, DiffTuple

from .service import ServiceBack, ServiceTuple
from .topic import TopicBack, TopicTuple
from .param import ParamBack, ParamTuple

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None


class RosInterface(BaseInterface):

    """
    RosInterface.
    """
    def __init__(self, node_name, services=None, topics=None, params=None, enable_cache=False, argv=None):
        # This runs in a child process (managed by PyrosROS) and as a normal ros node)
        # First thing to do : initialize the ros node and disable signals to avoid overriding callers behavior
        rospy.init_node(node_name, argv=argv, disable_signals=True)
        rospy.loginfo('RosInterface {name} node started with args : {argv}'.format(name=node_name, argv=argv))
        # note on dynamic update (config reload, etc.) this is reinitialized
        # rospy.init_node supports being reinitialized with the exact same arguments

        self.enable_cache = enable_cache
        if self.enable_cache and rocon_python_comms is None:
            rospy.logerr("Connection Cache enabled for RosInterface, but rocon_python_comms not found. Disabling.")
            self.enable_cache = False

        self.cb_lock = threading.Lock()
        with self.cb_lock:
            self.cb_ss = Queue.Queue()
            self.cb_ss_dt = Queue.Queue()

        # we add params from ROS environment, if we get something there (bwcompat behavior)
        services = services or []
        topics = topics or []
        params = params or []
        services += list(set(ast.literal_eval(rospy.get_param('~services', "[]"))))
        topics += list(set(ast.literal_eval(rospy.get_param('~topics', "[]"))))
        params += list(set(ast.literal_eval(rospy.get_param('~params', "[]"))))
        enable_cache = rospy.get_param('~enable_cache', enable_cache)

        if enable_cache is not None:
            self.enable_cache = enable_cache
        # Note : None means no change ( different from [] )
        rospy.loginfo("""[{name}] ROS Interface initialized with:
        -    services : {services}
        -    topics : {topics}
        -    params : {params}
        -    enable_cache : {enable_cache}
        -        """.format(name=__name__,
                            topics="\n" + "- ".rjust(10) + "\n\t- ".join(topics) if topics else [],
                            services="\n" + "- ".rjust(10) + "\n\t- ".join(services) if services else [],
                            params="\n" + "- ".rjust(10) + "\n\t- ".join(params) if params else [],
                            enable_cache=enable_cache)
                      )

        # This base constructor assumes the system to interface with is already available ( can do a get_svc_list() )
        super(RosInterface, self).__init__(services or [], topics or [], params or [])

        # connecting to the master via proxy object
        self._master = rospy.get_master()

        #: If enabled, connection cache proxy will be setup in update() to allow dynamic update via config.
        # TODO : double check : maybe useless now since we completely reinit the interface for dynamic update...
        self.connection_cache = None

        # Setting our list of interfaced topic right when we start
        rospy.set_param('~' + TopicBack.IF_TOPIC_PARAM, [])

        # Setup our debug log
        # We need this debug log since rospy.logdebug does NOT store debug messages in the log.
        # But it should Ref : http://wiki.ros.org/rospy/Overview/Logging#Reading_log_messages
        # TODO : find why.
        ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser("~"), '.ros'))
        self._debug_logger = logging.getLogger('pyros.ros_interface')
        if not os.path.exists(os.path.join(ros_home, 'logdebug')):
            os.makedirs(os.path.join(ros_home, 'logdebug'))
        logfilename = os.path.join(ros_home, 'logdebug', rospy.get_name()[1:].replace(os.path.sep, "-") + '_pyros_rosinterface.log')
        file_handler = logging.handlers.RotatingFileHandler(
            logfilename,
            maxBytes=100 * 131072,
            backupCount=10
        )
        self._debug_logger.propagate = False  # to avoid propagating to root logger
        self._debug_logger.setLevel(logging.DEBUG)
        self._debug_logger.addHandler(file_handler)
    # ros functions that should connect with the ros system we want to interface with
    # SERVICES
    def get_svc_list(self):  # function returning all services available on the system
        return [s for s in self.services_available]

    def service_type_resolver(self, service_name):  # function resolving the type of a service
        # get first matching service
        svc = self.services_available.get(service_name)
        if svc:
            if svc.type is None:  # if the type is unknown, lets discover it
                try:
                    resolved_service_name = rospy.resolve_name(service_name)  # required or not ?
                    svc.type = rosservice.get_service_type(resolved_service_name)
                except rosservice.ROSServiceIOException:  # exception can occur -> just reraise
                    raise
            return svc.type  # return the type
        else:
            rospy.logerr("ERROR while resolving {service_name}. Service not known as available. Ignoring".format(**locals()))
            return None

    def ServiceMaker(self, service_name, service_type):  # the service class implementation
        return ServiceBack(service_name, service_type)

    def ServiceCleaner(self, service):  # the service class cleanup implementation
        return service.cleanup()

    # TOPICS
    def get_topic_list(self):  # function returning all topics available on the system
        return [t for t in self.topics_available]

    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        # get first matching service
        tpc = self.topics_available.get(topic_name)
        if tpc:
            if tpc.type is None:  # if the type is unknown, lets discover it
                try:
                    resolved_topic_name = rospy.resolve_name(topic_name)
                    tpc.type, _, _ = rostopic.get_topic_type(resolved_topic_name)
                except rosservice.ROSServiceIOException:  # exception can occur -> just reraise
                   raise
            return tpc.type  # return the first we find. enough.
        else:
            rospy.logerr("ERROR while resolving {topic_name}. Topic not known as available. Ignoring".format(**locals()))
            return None

    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return TopicBack(topic_name, topic_type, *args, **kwargs)

    def TopicCleaner(self, topic):  # the topic class implementation
        return topic.cleanup()

    # PARAMS
    def get_param_list(self):  # function returning all params available on the system
        return [p for p in self.params_available]

    def param_type_resolver(self, param_name):  # function resolving the type of a param
        prm = self.params_available.get(param_name)
        if prm:
            if prm.type is None:  # if the type is unknown, lets discover it (since the service is supposed to exist)
                # TODO : param master API
                pass
            return prm.type  # return the first we find. enough.
        else:
            rospy.logerr("ERROR while resolving {param_name}. Param not known as available. Ignoring".format(**locals()))
            return None

    def ParamMaker(self, param_name, param_type):  # the param class implementation
        return ParamBack(param_name, param_type)

    def ParamCleaner(self, param):  # the param class implementation
        return param.cleanup()

    def _get_pyros_topics(self):
        if_topics = {}
        for par in self.params_available:
            if par.endswith(TopicBack.IF_TOPIC_PARAM):
                # extract process name from param name ( removing extra slash )
                pname = par[:- len('/' + TopicBack.IF_TOPIC_PARAM)]
                if_topics[pname] = rospy.get_param(par, [])
        return  if_topics

    def _filter_out_pyros_topics(self, publishers, subscribers, if_topics=None):
        """
        This method filter out the topic pubs / subs that are due to pyros behavior itself.
        These extra pubs/subs should not be used to represent the state of the system we connect to.
        :param publishers:
        :param subscribers:
        :param if_topics: topics that are there because of pyros interface (such as provided from _get_pyros_topics)
        :return:
        """
        # getting the list of interfaced topics from well known node param
        if_topics = if_topics or self._get_pyros_topics()

        # Examination of topics :
        # We keep publishers that are provided by something else ( not our exposed topic pub if present )
        # OR if we have locally multiple pubs / subs.
        filtered_publishers = []
        for p in publishers:
            # keeping only nodes that are not pyros interface for this topic
            # when added pub, also keeping interface nodes that have more than one interface (useful for tests and nodelets, etc. )
            # TODO : 1 here is a magic number. we should get_num_connections() instead
            nonif_pub_providers = [pp for pp in p[1] if (p[0] not in if_topics.get(pp, []) or TopicBack.more_than_pub_interface_added(p[0], 1))]
            if nonif_pub_providers:
                filtered_publishers.append([p[0], nonif_pub_providers])

        # We keep subscribers that are provided by something else ( not our exposed topic sub if present )
        # OR if we have locally multiple pubs / subs.
        filtered_subscribers = []
        for s in subscribers:
            # keeping only nodes that are not pyros interface for this topic
            # when added sub, also keeping interface nodes that have more than one interface (useful for tests and nodelets, etc. )
            # TODO : 1 here is a magic number. we should get_num_connections() instead
            nonif_sub_providers = [sp for sp in s[1] if (s[0] not in if_topics.get(sp, []) or TopicBack.more_than_sub_interface_added(s[0], 1))]
            if nonif_sub_providers:
                filtered_subscribers.append([s[0], nonif_sub_providers])

        return filtered_publishers, filtered_subscribers

    # This is really needed because an interfaced topic satisfies itself, and no diff message will be received
    # from ROS, even if the topic is gone. This is because the interface pub still requires the topic.
    # This returns the list of topics that satisfy themselves, to add them to the diff
    def get_lone_interfaced_topics(self):
        """
        Returns lone interfaced topic in the same way as it would be returned by the ROS system.
        But reading only internal data, since the local interface state (last pub/sub) will trigger the change.
        :return:
        """
        lone_topics = []
        for tname, t in self.topics.iteritems():
            if TopicBack.is_pub_interface_last(tname, t.pub.get_num_connections()) and TopicBack.is_sub_interface_last(tname, t.sub.get_num_connections()):
                lone_topics.append([tname, [rospy.get_name()]])
                self.topics_available.pop(tname)  # without this, the topic will remain in self.topics_available until the cache node can update, etc. -> delay
        return lone_topics

    def retrieve_params(self):
        """
        called to update params from rospy.
        CAREFUL : this can be called from another thread (subscriber callback)
        """
        params = rospy.get_param_names()
        self.reset_params(params)

    def reset_params(self, params):
        """
        called to update params from rospy.
        CAREFUL : this can be called from another thread (subscriber callback)
        """
        with self.params_available_lock:
            self.params_available = dict()
            for p in params:
                pt = []
                ptp = ParamTuple(name=p, type=pt[1] if len(pt) > 0 else None)
                self.params_available[ptp.name] = ptp

    def compute_params(self, params_dt):
        """
        called to update params from rospy.
        CAREFUL : this can be called from another thread (subscriber callback)
        """
        with self.params_available_lock:
            for p in self.params_available:
                pt = ParamTuple(name=p[0], type=None)
                if pt.name in self.params_available:
                    if self.params_available[pt.name].type is None or pt.type is not None:
                        self.params_available[pt.name].type = pt.type
                    else:
                        self.params_available[pt.name] = pt

            for p in self.params_available:
                pt = ParamTuple(name=p[0], type=None)
                if pt.name in self.params_available:
                    self.params_available.pop(pt.name, None)

        return params_dt

    def retrieve_system_state(self):
        """
        This will retrieve the system state from ROS master if needed, and apply changes to local variable to keep
        a local representation of the connections available up to date.
        """
        try:
            # we call the master only if we dont get system_state from connection cache
            if self.enable_cache and self.connection_cache is not None:
                publishers, subscribers, services = self.connection_cache.getSystemState()
                topic_types = self.connection_cache.getTopicTypes()
                try:
                    service_types = self.connection_cache.getServiceTypes()
                    # handling fallback here since master doesnt have the API
                except rocon_python_comms.UnknownSystemState as exc:
                    service_types = []
            else:
                publishers, subscribers, services = self._master.getSystemState()[2]
                topic_types = self._master.getTopicTypes()[2]
                service_types = []  # master misses this API to be consistent

            self.reset_system_state(publishers, subscribers, services, topic_types, service_types)

        except socket.error:
            rospy.logerr("[{name}] couldn't get system state from the master ".format(name=__name__))

    def reset_system_state(self, publishers, subscribers, services, topic_types, service_types):
        """
        Reset internal system state representation.
        expect lists in format similar to masterAPI.
        :param publishers:
        :param subscribers:
        :param services:
        :param topic_types:
        :param service_types:
        :return:
        """
        iftopics = self._get_pyros_topics()
        filtered_publishers, filtered_subscribers = self._filter_out_pyros_topics(publishers, subscribers, if_topics=iftopics)
        # this is used with full list : we only need to filter out from that list.

        # We merge both pubs and subs, so that only one pub or one sub which is not ours is enough to keep the topic
        with self.topics_available_lock:
            self.topics_available = dict()
            for t in (filtered_publishers + filtered_subscribers):
                tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types), [])
                ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
                self.topics_available[ttp.name] = ttp

        with self.services_available_lock:
            self.services_available = dict()
            for s in services:
                st = next(ifilter(lambda lst: s[0] == lst[0], service_types), [])
                stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
                self.services_available[stp.name] = stp

        # We still need to return DiffTuples
        return services, filtered_publishers + filtered_subscribers

    def compute_system_state(self, publishers_dt, subscribers_dt, services_dt, topic_types_dt, service_types_dt):
        """
        This is called only if there is a cache proxy with a callback, and expects DiffTuple filled up with names or types
        :param services_dt:
        :param publishers_dt:
        :param subscribers_dt:
        :return:
        """
        iftopics = self._get_pyros_topics()
        filtered_added_publishers, filtered_added_subscribers = self._filter_out_pyros_topics(publishers_dt.added, subscribers_dt.added, if_topics=iftopics)
        # this is called with difference tuples
        # we need to add removed topics that are only exposed by this pyros interface.
        filtered_removed_publishers, filtered_removed_subscribers = self._filter_out_pyros_topics(publishers_dt.removed, subscribers_dt.removed, if_topics=iftopics)

        # collapsing add / remove pairs with the help of nodes multiplicity
        # here to avoid more complicated side effects later on
        added_pubs = {pub[0]: pub[1] for pub in filtered_added_publishers}
        added_subs = {sub[0]: sub[1] for sub in filtered_added_subscribers}
        removed_pubs = {pub[0]: pub[1] for pub in filtered_removed_publishers}
        removed_subs = {sub[0]: sub[1] for sub in filtered_removed_subscribers}

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

        # We merge both pubs and subs, so that only one pub or one sub which is not ours is enough to keep the topic
        # Need to be careful if pub and sub are added/removed at same time : only one topic added/removed
        added_topics = {pub[0]: pub[1] for pub in filtered_added_publishers}
        removed_topics = {pub[0]: pub[1] for pub in filtered_removed_publishers}

        for t in filtered_added_subscribers:
            added_topics[t[0]] = added_topics.get(t[0], []) + t[1]
        for t in filtered_removed_subscribers:
            removed_topics[t[0]] = removed_topics.get(t[0], []) + t[1]

        # TODO : improve here to make sure of nodes unicity after this collapsing step

        topics_dt = DiffTuple(
            added=[[k, v] for k, v in added_topics.iteritems()],
            removed=[[k, v] for k, v in removed_topics.iteritems()]
        )

        with self.topics_available_lock:
            for t in topics_dt.added:
                tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.added), [])
                ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
                if ttp.name in self.topics_available:
                    # if already available, we only update the endpoints list
                    self.topics_available[ttp.name].endpoints |= ttp.endpoints
                else:
                    self.topics_available[ttp.name] = ttp

            for t in topics_dt.removed:
                tt = next(ifilter(lambda ltt: t[0] == ltt[0], topic_types_dt.removed), [])
                ttp = TopicTuple(name=t[0], type=tt[1] if len(tt) > 0 else None, endpoints=set(t[1]))
                if ttp.name in self.topics_available:
                    self.topics_available[ttp.name].endpoints -= ttp.endpoints
                    if not self.topics_available[ttp.name].endpoints:
                        self.topics_available.pop(ttp.name, None)

        with self.services_available_lock:
            for s in services_dt.added:
                st = next(ifilter(lambda lst: s[0] == lst[0], service_types_dt.added), [])
                stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
                if stp.name in self.services_available:
                    if self.services_available[stp.name].type is None and stp.type is not None:
                        self.services_available[stp.name].type = stp.type
                else:
                    self.services_available[stp.name] = stp

            for s in services_dt.removed:
                st = next(ifilter(lambda lst: s[0] == lst[0], service_types_dt.removed), [])
                stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
                if stp.name in self.services_available:
                    self.services_available.pop(stp.name, None)

        # We still need to return DiffTuples
        return services_dt, topics_dt

    def update(self, system_state=None):

        # Destroying connection cache proxy if needed
        if self.connection_cache is not None and not self.enable_cache:
            # removing existing connection cache proxy to force a reinit of everything
            # to make sure we dont get a messed up system state with wrong list/diff from
            # dynamically switching cache on and off.
            self.connection_cache = None

        # TODO Instead of one or the other, we should have "two layer behaviors" with different frequencies
        # Fast loop checking only diff
        # Slow loop checking full state
        # It will allow recovering from any mistakes because of wrong diffs (update speed/race conditions/etc.)
        if self.enable_cache:
            if self.connection_cache is None:  # Building Connection Cache proxy if needed
                # connectioncache proxy if available (remap the topics if necessary instead of passing params)
                try:
                    self.connection_cache = rocon_python_comms.ConnectionCacheProxy(
                        list_sub='~connections_list',
                        handle_actions=False,
                        user_callback=self._proxy_cb,
                        diff_opt=True,
                        diff_sub='~connections_diff'
                    )
                except rocon_python_comms.ConnectionCacheProxy.InitializationTimeout as timeout_exc:
                    # timeout initializing : disabling the feature but we should be LOUD about it
                    rospy.logwarn("Pyros.rosinterface : FAILED during initialization of Connection Cache Proxy. Disabling.")
                    self.enable_cache = False
                else:
                    rospy.loginfo("Pyros.rosinterface : Connection Cache Optimization enabled")

            # determining params diff despite lack of API
            params = set(rospy.get_param_names())
            params_dt = DiffTuple(
                added=[p for p in params if p not in [pname for pname in self.params_available]],
                removed=[p for p in self.params_available if p not in [ifilter(lambda pf: pf == p, params)]]
            )
            params_dt = self.compute_params(params_dt)

            # detecting lone topics and simulating a removed detection to trigger remove from interface
            # These are only used for detection.
            # compute_system_state will retrieve them again after filtering out interface topics.
            # TODO : simplify and solidify logic for this case
            early_topics_dt = DiffTuple([], [t[0] for t in self.get_lone_interfaced_topics()])

            if early_topics_dt.added or early_topics_dt.removed:
                self._debug_logger.debug(rospy.get_name() + " Pyros.rosinterface : Early Topics Delta {early_topics_dt}".format(**locals()))

            # If we have a callback setup we process the diff we got since last time
            if (len(early_topics_dt.added) > 0 or len(early_topics_dt.removed) > 0) or (len(params_dt.added) > 0 or len(params_dt.removed) > 0) or self.cb_ss.qsize() > 0:

                # This will be set if we need to ignore current state, and reset it from list
                reset = False

                added_services = dict()
                removed_services = dict()
                added_publishers = dict()
                removed_publishers = dict()
                added_subscribers = dict()
                removed_subscribers = dict()

                added_topic_types = []
                removed_topic_types = []
                added_service_types = []
                removed_service_types = []

                with self.cb_lock:
                    stop = False
                    while not stop:  # TODO we need to make sure we consume faster than we get fed.
                        try:
                            cb_ss = self.cb_ss.get_nowait()
                            cb_ss_dt = self.cb_ss_dt.get_nowait()

                            # if there was no change but we got a callback,
                            # it means it s the first and we need to set the whole list
                            if cb_ss_dt.added is None and cb_ss_dt.removed is None:
                                # we need to break here already and reset
                                # and the previous diff we got dont matter any longer

                                for k, v in cb_ss.services.iteritems():
                                    added_services[k] = added_services.get(k, set()) | v.nodes

                                for k, v in cb_ss.publishers.iteritems():
                                    added_publishers[k] = added_publishers.get(k, set()) | v.nodes

                                for k, v in cb_ss.services.iteritems():
                                    added_subscribers[k] = added_subscribers.get(k, set()) | v.nodes

                                pubset = {(name, chan.type) for name, chan in cb_ss.publishers.iteritems()}
                                subset = {(name, chan.type) for name, chan in cb_ss.subscribers.iteritems()}
                                added_topic_types = [list(t) for t in (pubset | subset)]

                                svcset = {(name, chan.type) for name, chan in cb_ss.services.iteritems()}
                                added_service_types = [list(t) for t in svcset]

                                reset = True
                                stop = True

                            else:
                                for k, v in cb_ss_dt.added.services.iteritems():
                                    added_services[k] = added_services.get(k, set()) | v.nodes
                                for k, v in cb_ss_dt.removed.services.iteritems():
                                    removed_services[k] = removed_services.get(k, set()) | v.nodes

                                for k, v in cb_ss_dt.added.publishers.iteritems():
                                    added_publishers[k] = added_publishers.get(k, set()) | v.nodes
                                for k, v in cb_ss_dt.removed.publishers.iteritems():
                                    removed_publishers[k] = removed_publishers.get(k, set()) | v.nodes

                                for k, v in cb_ss_dt.added.subscribers.iteritems():
                                    added_subscribers[k] = added_subscribers.get(k, set()) | v.nodes
                                for k, v in cb_ss_dt.removed.subscribers.iteritems():
                                    removed_subscribers[k] = removed_subscribers.get(k, set()) | v.nodes

                                # Careful here the previous loop produced result that still matters
                                pubset = {(name, chan.type) for name, chan in cb_ss_dt.added.publishers.iteritems()}
                                subset = {(name, chan.type) for name, chan in cb_ss_dt.added.subscribers.iteritems()}
                                added_topic_types += [list(t) for t in (pubset | subset)]

                                pubset = {(name, chan.type) for name, chan in cb_ss_dt.removed.publishers.iteritems()}
                                subset = {(name, chan.type) for name, chan in cb_ss_dt.removed.subscribers.iteritems()}
                                removed_topic_types += [list(t) for t in (pubset | subset)]

                                svcset = {(name, chan.type) for name, chan in cb_ss_dt.added.services.iteritems()}
                                added_service_types += [list(t) for t in svcset]

                                svcset = {(name, chan.type) for name, chan in cb_ss_dt.removed.services.iteritems()}
                                removed_service_types += [list(t) for t in svcset]

                        except Queue.Empty as exc:
                            # we re done here
                            stop = True

                # if we need to reset we do it right now and return.
                if reset:
                    # we will remove what we have now.
                    self.reset_params(params)
                    self.reset_system_state(  # here we need to get only the nodes' names
                            [[k, [n[0] for n in nset]] for k, nset in added_publishers.iteritems()],
                            [[k, [n[0] for n in nset]] for k, nset in added_subscribers.iteritems()],
                            [[k, [n[0] for n in nset]] for k, nset in added_services.iteritems()],
                            added_topic_types,
                            added_service_types
                    )
                    # we still need to return a diff to report on our behavior
                    # update() will compute diffs and do the job for us
                    dt = super(RosInterface, self).update()
                else:  # if we have any change, we process it
                    # converting data format. Here we want only the names/keys.
                    # Resolving the details will be done as usual

                    # here we need to get only the nodes' names
                    services_dt = DiffTuple(
                        added=[[k, [n[0] for n in nset]] for k, nset in added_services.iteritems()],
                        removed=[[k, [n[0] for n in nset]] for k, nset in removed_services.iteritems()]
                    )
                    # remove publishers added which are not new, and lost publisher which are still there
                    publishers_dt = DiffTuple(
                        added=[[k, [n[0] for n in nset]] for k, nset in added_publishers.iteritems()],
                        removed=[[k, [n[0] for n in nset]] for k, nset in removed_publishers.iteritems()]
                    )
                    subscribers_dt = DiffTuple(
                        added=[[k, [n[0] for n in nset]] for k, nset in added_subscribers.iteritems()],
                        removed=[[k, [n[0] for n in nset]] for k, nset in removed_subscribers.iteritems()]
                    )
                    topic_types_dt = DiffTuple(
                        added=added_topic_types,
                        removed=removed_topic_types
                    )
                    service_types_dt = DiffTuple(
                        added=added_service_types,
                        removed=removed_service_types
                    )
                    services_dt, topics_dt = self.compute_system_state(publishers_dt, subscribers_dt, services_dt, topic_types_dt, service_types_dt)

                    if topics_dt.added or topics_dt.removed:
                        self._debug_logger.debug(rospy.get_name() + " Pyros.rosinterface : Topics Delta {topics_dt}".format(**locals()))
                    if services_dt.added or services_dt.removed:
                        self._debug_logger.debug(rospy.get_name() + " Pyros.rosinterface : Services Delta {services_dt}".format(**locals()))

                    # update_on_diff wants only names
                    dt = super(RosInterface, self).update_on_diff(
                            DiffTuple([s[0] for s in services_dt.added], [s[0] for s in services_dt.removed]),
                            DiffTuple([t[0] for t in topics_dt.added] + early_topics_dt.added, [t[0] for t in topics_dt.removed] + early_topics_dt.removed),
                            DiffTuple([p[0] for p in params_dt.added], [p[0] for p in params_dt.removed])
                    )
                if dt.added or dt.removed:
                    self._debug_logger.debug(rospy.get_name() + " Pyros.rosinterface : Interface Update Delta {dt}".format(**locals()))
                return dt
            else:
                # no update : nothing to do
                return DiffTuple([], [])

        else:  # default retrieve full system state (cache or master otherwise)
            self.retrieve_params()
            self.retrieve_system_state()  # This will call the master if needed
            return super(RosInterface, self).update()

    def _proxy_cb(self, system_state, added_system_state, lost_system_state):
        with self.cb_lock:
            self.cb_ss.put(system_state)

            self.cb_ss_dt.put(DiffTuple(
                added=added_system_state,
                removed=lost_system_state
            ))

BaseInterface.register(RosInterface)


