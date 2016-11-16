from __future__ import absolute_import
from __future__ import print_function

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

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

from ..baseinterface import DiffTuple
from .baseinterface import BaseInterface
from .param_if_pool import RosParamIfPool
from .service_if_pool import RosServiceIfPool
from .subscriber_if_pool import RosSubscriberIfPool
from .publisher_if_pool import RosPublisherIfPool

from .connection_cache_utils import connection_cache_proxy_create, connection_cache_marshall, connection_cache_merge_marshalled

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None

# To make sure things dont get messed up between threads
CacheTuple = (namedtuple("CacheTuple", "complete added removed"))


class RosInterface(BaseInterface):

    """
    RosInterface.
    """
    def __init__(self, node_name, publishers=None, subscribers=None, services=None, params=None, enable_cache=False, argv=None):
        # This runs in a child process (managed by PyrosROS) and as a normal ros node)

        # First thing to do : find the rosmaster...
        # master has to be running here or we just wait for ever
        # TODO : improve...
        m, _ = pyros_utils.get_master(spawn=False)
        while not m.is_online():
            _logger.warning("ROSMASTER not found !!!...")
            time.sleep(0.5)

        # Second thing to do : initialize the ros node and disable signals to avoid overriding callers behavior
        rospy.init_node(node_name, argv=argv, disable_signals=True)
        rospy.loginfo('RosInterface {name} node started with args : {argv}'.format(name=node_name, argv=argv))
        # note on dynamic update (config reload, etc.) this is reinitialized
        # rospy.init_node supports being reinitialized with the exact same arguments

        self.enable_cache = enable_cache
        if self.enable_cache and rocon_python_comms is None:
            rospy.logerr("Connection Cache enabled for RosInterface, but rocon_python_comms not found. Disabling.")
            self.enable_cache = False

        # only one queue to avoid sync issues
        self.cb_ss = Queue.Queue()

        # we add params from ROS environment, if we get something there (bwcompat behavior)
        services = services or []
        publishers = publishers or []
        subscribers = subscribers or []
        params = params or []
        services += list(set(ast.literal_eval(rospy.get_param('~services', "[]"))))

        # bwcompat
        publishers += list(set(ast.literal_eval(rospy.get_param('~topics', "[]"))))
        subscribers += list(set(ast.literal_eval(rospy.get_param('~topics', "[]"))))
        # new
        publishers += list(set(ast.literal_eval(rospy.get_param('~publishers', "[]"))))
        subscribers += list(set(ast.literal_eval(rospy.get_param('~subscribers', "[]"))))

        params += list(set(ast.literal_eval(rospy.get_param('~params', "[]"))))
        enable_cache = rospy.get_param('~enable_cache', enable_cache)

        if enable_cache is not None:
            self.enable_cache = enable_cache
        # Note : None means no change ( different from [] )
        rospy.loginfo("""[{name}] ROS Interface initialized with:
        -    services : {services}
        -    publishers : {publishers}
        -    subscribers : {subscribers}
        -    params : {params}
        -    enable_cache : {enable_cache}
        """.format(
            name=__name__,
            publishers="\n" + "- ".rjust(10) + "\n\t- ".join(publishers) if publishers else [],
            subscribers="\n" + "- ".rjust(10) + "\n\t- ".join(subscribers) if subscribers else [],
            services="\n" + "- ".rjust(10) + "\n\t- ".join(services) if services else [],
            params="\n" + "- ".rjust(10) + "\n\t- ".join(params) if params else [],
            enable_cache=enable_cache)
        )

        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        params_pool = RosParamIfPool(params)
        services_pool = RosServiceIfPool(services)
        subscribers_pool = RosSubscriberIfPool(subscribers)
        publishers_pool = RosPublisherIfPool(publishers)

        super(RosInterface, self).__init__(publishers_pool, subscribers_pool, services_pool, params_pool)

        # connecting to the master via proxy object
        self._master = rospy.get_master()

        #: If enabled, connection cache proxy will be setup in update() to allow dynamic update via config.
        # TODO : double check : maybe useless now since we completely reinit the interface for dynamic update...
        self.connection_cache = None

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

            # Getting this doesnt depend on cache for now
            params = set(rospy.get_param_names())

            return publishers, subscribers, services, params, topic_types, service_types

        except socket.error:
            rospy.logerr("[{name}] couldn't get system state from the master ".format(name=__name__))

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    #@profile
    def update(self):

        backedup_complete_cb_ss = None

        #update will retrieve system state here
        publishers = []
        subscribers = []
        services = []
        params = []
        topic_types = []
        service_types = []

        # and populate these to represent the changes done on the interface
        params_if_dt = DiffTuple([], [])
        services_if_dt = DiffTuple([], [])
        subscribers_if_dt = DiffTuple([], [])
        publishers_if_dt = DiffTuple([], [])

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
            if self.connection_cache is None:  # Building Connection Cache Proxy if needed
                self.connection_cache = connection_cache_proxy_create(self._proxy_cb)
                if self.connection_cache:
                    self.enable_cache = True
                else:
                    self.enable_cache = False

        # TMP until it s implemented in the connection cache
        # Because the cache doesnt currently do it
        params = set(rospy.get_param_names())
        # determining params diff despite lack of API
        params_dt = DiffTuple(
            added=[p for p in params if p not in self.params_available],
            removed=[p for p in self.params_available if p not in params]
        )

        # If we have the connection_cache and a callback setup we process the diff (and maybe param changes)
        if self.connection_cache and (params_dt.added or params_dt.removed or backedup_complete_cb_ss is not None or self.cb_ss.qsize() > 0):
            try:
                if backedup_complete_cb_ss is not None:
                    # using the message backed up after latest diff process
                    cb_ss = backedup_complete_cb_ss
                    backedup_complete_cb_ss = None  #CAREFUL : untested...
                else:
                    cb_ss = self.cb_ss.get_nowait()
                # print("CC MSG !")  # if we didn't except on empty queue, so we got a message

            except Queue.Empty:

                return self.update_nodelta(params_dt)

            else:
                # if there was no change,
                # it means it s the first and we need to initialize with the full list
                if cb_ss.added is None and cb_ss.removed is None:
                    #print("CC COMPLETE !")
                    publishers = cb_ss.complete.get('publishers', [])
                    subscribers = cb_ss.complete.get('subscribers', [])
                    services = cb_ss.complete.get('services', [])
                    # NOT YET...
                    #params = cb_ss.complete.get('params', params)
                    topic_types = cb_ss.complete.get('topic_types', [])
                    service_types = cb_ss.complete.get('service_types', [])

                    # To convert to ROS master API format :
                    publishers = [[p, [n[0] for n in nset]] for p, nset in publishers.iteritems()]
                    subscribers = [[s, [n[0] for n in nset]] for s, nset in subscribers.iteritems()]
                    services = [[s, [n[0] for n in nset]] for s, nset in services.iteritems()]

                    #print("CC COMPLETEDDDD !")

                    # we go back to normal flow
                    #print("UPDATE FULLSTATE ON CACHE LIST")
                    return self.update_fullstate(publishers, subscribers, services, params, topic_types, service_types)

                else:  # we have a delta, we can use it directly and skip the rest

                    #print("CC DELTA !")
                    next_cb_ss = cb_ss
                    while self.cb_ss.qsize() > 0 and not (next_cb_ss.added is None and next_cb_ss is None):
                        # merging multiple diff messages as fast as possible, until the next complete status
                        next_cb_ss = self.cb_ss.get_nowait()
                        if not (next_cb_ss.added is None and next_cb_ss is None):
                            cb_ss = CacheTuple(
                                complete=None,  # this is ignored here
                                # CAREFUL one of added or remove can still be None here...
                                added=connection_cache_merge_marshalled(next_cb_ss.added or {}, cb_ss.added),
                                removed=connection_cache_merge_marshalled(next_cb_ss.removed or {}, cb_ss.removed)
                            )
                        else:  # we need to pass next_cb_ss to handle complete list again...
                            backedup_complete_cb_ss = next_cb_ss

                    added_publishers = cb_ss.added.get('publishers', [])
                    added_subscribers = cb_ss.added.get('subscribers', [])
                    added_services = cb_ss.added.get('services', [])
                    # NOT YET
                    #added_params = cb_ss.added.get('params', params_dt.added)
                    added_params = params_dt.added
                    added_topic_types = cb_ss.added.get('topic_types', [])
                    added_service_types = cb_ss.added.get('service_types', [])

                    removed_publishers = cb_ss.removed.get('publishers', [])
                    removed_subscribers = cb_ss.removed.get('subscribers', [])
                    removed_services = cb_ss.removed.get('services', [])
                    # NOT YET
                    #removed_params = cb_ss.removed.get('params', params_dt.removed)
                    removed_params = params_dt.removed
                    removed_topic_types = cb_ss.removed.get('topic_types', [])
                    removed_service_types = cb_ss.removed.get('service_types', [])

                    #print("UPDATE STATEDELTA ON CACHE DIFF")
                    return self.update_statedelta(
                        added_publishers, added_subscribers, added_services, added_params, added_topic_types, added_service_types,
                        removed_publishers, removed_subscribers, removed_services, removed_params, removed_topic_types, removed_service_types
                    )
        elif not self.connection_cache:  # make sure we are not using connection cache (otherwise state representations might be out of sync !!)
            #print("GETTING STATE FROM MASTER")
            publishers, subscribers, services, params, topic_types, service_types = self.retrieve_system_state()  # This will call the master

            #print("UPDATE FULLSTATE")
            # NOTE : we want to be certain here that we do not mix full state representation from master with representation from cache (out of sync !!!)
            return self.update_fullstate(publishers, subscribers, services, params, topic_types, service_types)
        else:
            # This happens when the update is triggered, we are supposed to use the cache, but we got no message from the connection cache proxy.
            # WE DO NOT WANT TO GET FULLSTATE FROM CACHE, since it is out of date (ex: removed publishers are not removed until next message...)
            # print("GETTING STATE FROM CACHE")
            # publishers, subscribers, services, params, topic_types, service_types = self.retrieve_system_state()  # This will call the cache
            #
            # print("UPDATE FULLSTATE")
            # return self.update_fullstate(publishers, subscribers, services, params, topic_types, service_types)
            # ==> We need to wait for next message...
            return self.update_nodelta(params_dt)

    def update_fullstate(self, publishers, subscribers, services, params, topic_types, service_types):
        # NORMAL full update
        self._debug_logger.debug("""SYSTEM STATE :
                    - publishers : {publishers}
                    - subscribers : {subscribers}
                    - services : {services}
                    - params : {params}
                    - topic_types : {topic_types}
                    - service_types : {service_types}
                """.format(**locals()))

        # TODO : unify with the reset behavior in case of cache...

        # Needs to be done first, since topic algorithm depends on it
        # print("PARAMS : {params}".format(**locals()))
        params_if_dt = self.params_if_pool.update(params=params)
        # print("PARAM IF DT : {params_if_dt}".format(**locals()))

        # print("SERVICES : {services}".format(**locals()))
        services_if_dt = self.services_if_pool.update(services, service_types)
        # print("SERVICE IF DT : {services_if_dt}".format(**locals()))

        # print("SUBSCRIBERS : {subscribers}".format(**locals()))
        subscribers_if_dt = self.subscribers_if_pool.update(subscribers, topic_types)
        # print("SUBSCRIBER IF DT : {subscribers_if_dt}".format(**locals()))

        # print("PUBLISHERS : {publishers}".format(**locals()))
        publishers_if_dt = self.publishers_if_pool.update(publishers, topic_types)
        # print("PUBLISHER IF DT : {publishers_if_dt}".format(**locals()))

        dt = DiffTuple(
            added=params_if_dt.added + services_if_dt.added + subscribers_if_dt.added + publishers_if_dt.added,
            removed=params_if_dt.removed + services_if_dt.removed + subscribers_if_dt.removed + publishers_if_dt.removed
        )

        self._debug_logger.debug("""
                    ROS INTERFACE ADDED : {dt.added}
                    ROS INTERFACE REMOVED : {dt.removed}
                """.format(**locals()))

        return dt

    def update_nodelta(self, params_dt):
        """
        Running updates of nothing (workaround until params are handled by cache)
        :param params_dt:
        :return:
        """
        # we re NOT done here, we might still need to update params
        params_if_dt = self.params_if_pool.update_delta(params_dt=params_dt)

        topic_types_dt = DiffTuple(
            added=[],
            removed=[]  # shouldnt matter
        )

        services_if_dt = DiffTuple([], [])
        # TMP : NOT dropping topics early (just be patient and wait for the cache callback to come...)
        # topics_if_dt = self.topics_pool.update_delta(topics_dt, topic_types_dt)
        subscribers_if_dt = DiffTuple([], [])
        publishers_if_dt = DiffTuple([], [])

        # and here we need to return to not do the normal full update
        dt = DiffTuple(
            added=params_if_dt.added + services_if_dt.added + subscribers_if_dt.added + publishers_if_dt.added,
            removed=params_if_dt.removed + services_if_dt.removed + subscribers_if_dt.removed + publishers_if_dt.removed
        )

        self._debug_logger.debug("""
                            ROS INTERFACE DIFF ADDED : {dt.added}
                            ROS INTERFACE DIFF REMOVED : {dt.removed}
                        """.format(**locals()))

        return dt

    def update_statedelta(
            self,
            added_publishers, added_subscribers, added_services, added_params, added_topic_types, added_service_types,
            removed_publishers, removed_subscribers, removed_services, removed_params, removed_topic_types, removed_service_types
    ):
        params_dt = DiffTuple(
            added=added_params,
            removed=removed_params
        )

        # Needs to be done first, since topic algorithm depends on it
        params_if_dt = self.params_if_pool.update_delta(params_dt=params_dt)

        # here we need to get only the nodes' names to match ROs master API format
        services_dt = DiffTuple(
            added=[[k, [n[0] for n in nset]] for k, nset in added_services.iteritems()],
            removed=[[k, [n[0] for n in nset]] for k, nset in removed_services.iteritems()]
        )

        service_types_dt = DiffTuple(
            added=added_service_types,
            removed=removed_service_types
        )

        services_if_dt = self.services_if_pool.update_delta(services_dt, service_types_dt)

        # here we need to get only the nodes' names to match ROs master API format
        # CAREFUL about unicity here, a Pub|Sub can have multiple endpoint within the same node...
        # We do not care about that information. If we have to care, then we should pass also the uri...
        publishers_dt = DiffTuple(
            added=[[k, [n[0] for n in set(nset)]] for k, nset in added_publishers.iteritems()],
            removed=[[k, [n[0] for n in set(nset)]] for k, nset in removed_publishers.iteritems()]
        )
        subscribers_dt = DiffTuple(
            added=[[k, [n[0] for n in set(nset)]] for k, nset in added_subscribers.iteritems()],
            removed=[[k, [n[0] for n in set(nset)]] for k, nset in removed_subscribers.iteritems()]
        )

        # NOW DONE IN update_delta
        # # CAREFUL, topic interface by itself also makes the topic detected on system
        # # Check if there are any pyros interface with it and ignore them
        # topics_to_drop = TopicBack.get_interface_only_topics()
        # # TODO : simplify : same as early_topics_to_drop ?
        #
        # # filtering the topic dict
        # for td, tnode in added_topics.iteritems():
        #     tnode = [n for n in tnode if n not in topics_to_drop.get(td, [])]
        #
        # added_topics_list = [[td, added_topics[td]] for td in added_topics if added_topics[td]]  # filtering out topics with no endpoints
        #
        # # we also need to simulate topic removal here (only names), to trigger a cleanup of interface if it s last one
        # removed_topics_list = [[td, removed_topics[td]] for td in removed_topics] + early_topics_to_drop

        topic_types_dt = DiffTuple(
            added=added_topic_types,
            removed=removed_topic_types
        )

        subscribers_if_dt = self.subscribers_if_pool.update_delta(subscribers_dt, topic_types_dt)
        publishers_if_dt = self.publishers_if_pool.update_delta(publishers_dt, topic_types_dt)

        if publishers_if_dt.added or publishers_if_dt.removed:
            self._debug_logger.debug(
                rospy.get_name() + " Pyros.rosinterface : Publishers Delta {publishers_if_dt}".format(
                    **locals()))
        if subscribers_if_dt.added or subscribers_if_dt.removed:
            self._debug_logger.debug(
                rospy.get_name() + " Pyros.rosinterface : Subscribers Delta {subscribers_if_dt}".format(
                    **locals()))
        if services_if_dt.added or services_if_dt.removed:
            self._debug_logger.debug(
                rospy.get_name() + " Pyros.rosinterface : Services Delta {services_if_dt}".format(
                    **locals()))

        # TODO : put that in debug log and show based on python logger configuration
        # print("Pyros ROS interface UPDATE")
        # print("Params ADDED : {0}".format([p for p in params_dt.added]))
        # print("Params GONE : {0}".format([p for p in params_dt.removed]))
        # print("Topics ADDED : {0}".format([t[0] for t in topics_dt.added] + early_topics_dt.added))
        # print("Topics GONE : {0}".format([t[0] for t in topics_dt.removed] + early_topics_dt.removed))
        # print("Srvs ADDED: {0}".format([s[0] for s in services_dt.added]))
        # print("Srvs GONE: {0}".format([s[0] for s in services_dt.removed]))

        # update_on_diff wants only names
        # dt = super(RosInterface, self).update_on_diff(
        #         DiffTuple([s[0] for s in services_dt.added], [s[0] for s in services_dt.removed]),
        #         DiffTuple([t[0] for t in topics_dt.added] + early_topics_dt.added, [t[0] for t in topics_dt.removed] + early_topics_dt.removed),
        #         # Careful params_dt has a different content than service and topics, due to different ROS API
        #         # TODO : make this as uniform as possible
        #         DiffTuple([p for p in params_dt.added], [p for p in params_dt.removed])
        # )

        # and here we need to return to not do the normal full update
        dt = DiffTuple(
            added=params_if_dt.added + services_if_dt.added + subscribers_if_dt.added + publishers_if_dt.added,
            removed=params_if_dt.removed + services_if_dt.removed + subscribers_if_dt.removed + publishers_if_dt.removed
        )

        self._debug_logger.debug("""
                                        ROS INTERFACE DIFF ADDED : {dt.added}
                                        ROS INTERFACE DIFF REMOVED : {dt.removed}
                                    """.format(**locals()))

        return dt


    def _proxy_cb(self, system_state, added_system_state, lost_system_state):
        self.cb_ss.put(CacheTuple(
            complete=connection_cache_marshall(system_state),
            added=connection_cache_marshall(added_system_state) if added_system_state is not None else None,
            removed=connection_cache_marshall(lost_system_state) if lost_system_state is not None else None
        ))



