from __future__ import absolute_import

import logging
from .api import rospy_safe as rospy

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None


def connection_cache_proxy_create(_proxy_cb=None):
    """
    Creates and return a connection cache proxy if possible, handling all likely exceptions
    :param _proxy_cb:
    :return:
    """
    if rocon_python_comms is None:
        return None
    else:
        # connectioncache proxy if available (remap the topics if necessary instead of passing params)
        try:
            connection_cache_proxy = rocon_python_comms.ConnectionCacheProxy(
                list_sub='~connections_list',
                handle_actions=False,
                user_callback=_proxy_cb,
                diff_opt=True,
                diff_sub='~connections_diff'
            )

        except AttributeError as attr_exc:
            # attribute error (likely rocon_python_comms doesnt have ConnectionCacheProxy)
            # NOT EXPECTED System configuration problem : BE LOUD !
            # timeout initializing : disabling the feature but we should be LOUD about it
            rospy.logwarn("connection_cache_utils : FAILED during initialization of Connection Cache Proxy. Disabling.")
            import traceback
            rospy.logwarn('Exception: {0}'.format(traceback.format_stack()))
            return None

        except rocon_python_comms.ConnectionCacheProxy.InitializationTimeout as timeout_exc:

            # timeout initializing : disabling the feature but we should WARN about it
            rospy.logwarn("connection_cache_utils : TIMEOUT during initialization of Connection Cache Proxy. Disabling.")
            return None

        else:
            rospy.loginfo("connection_cache_utils : Connection Cache Optimization enabled")
            return connection_cache_proxy


def connection_cache_marshall(cb_ss):
    """
    Marshall data comming from connection cache callback to adapt to the internal data format for pyros
    :param cb_ss: system state to convert
    :param old_cb_ss: Old system state in case we want to merge with the recent one
    :return:
    """

    publishers = {}
    subscribers = {}
    topic_types = []
    services = {}
    service_types = []
    params = {}  # TODO cb_ss.params

    # We assume we get SystemState type from the connection cache proxy (directly, the first time)

    for k, v in cb_ss.services.iteritems():
        services[k] = services.get(k, set()) | v.nodes

    for k, v in cb_ss.publishers.iteritems():
        publishers[k] = publishers.get(k, set()) | v.nodes

    for k, v in cb_ss.subscribers.iteritems():
        subscribers[k] = subscribers.get(k, set()) | v.nodes

    pubset = {(name, chan.type) for name, chan in cb_ss.publishers.iteritems()}
    subset = {(name, chan.type) for name, chan in cb_ss.subscribers.iteritems()}
    topic_types += [list(t) for t in (pubset | subset)]

    svcset = {(name, chan.type) for name, chan in cb_ss.services.iteritems()}
    service_types += [list(t) for t in svcset]

    return {
        'publishers': publishers,
        'subscribers': subscribers,
        'services': services,
        'params': params,
        'topic_types': topic_types,
        'services_types': service_types
    }


def connection_cache_merge_marshalled(marshalled_ss, old_marshalled_ss=None):

    old_marshalled_ss = old_marshalled_ss or {}

    publishers = old_marshalled_ss.get('publishers', {})
    subscribers = old_marshalled_ss.get('subscribers', {})
    topic_types = old_marshalled_ss.get('topic_types', [])
    services = old_marshalled_ss.get('services', {})
    service_types = old_marshalled_ss.get('service_types', [])
    params = {}  # TODO cb_ss.params

    # We come back again here after marshalling already once...
    for k, endpoints in marshalled_ss.get('services', {}).iteritems():
        services[k] = services.get(k, set()) | endpoints

    for k, endpoints in marshalled_ss.get('publishers', {}).iteritems():
        publishers[k] = publishers.get(k, set()) | endpoints

    for k, endpoints in marshalled_ss.get('subscribers', {}).iteritems():
        subscribers[k] = subscribers.get(k, set()) | endpoints

    topic_types += marshalled_ss.get('topic_types', [])
    service_types += marshalled_ss.get('service_types', [])

    return {
        'publishers': publishers,
        'subscribers': subscribers,
        'services': services,
        'params': params,
        'topic_types': topic_types,
        'services_types': service_types
    }