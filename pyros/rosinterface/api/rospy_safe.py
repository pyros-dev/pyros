from __future__ import absolute_import
from __future__ import print_function

import socket
import time
import rospy


# We wrap rospy function into safeguards for socket error
# since the master seems to be quite sensitive to Network health.
# TODO : refine which ones need which exception handling...

retry_timeout = 1  # number of seconds to wait before retrying


def logerr(msg):
    res = None
    while res is None:
        try:
            res = rospy.logerr(msg)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling logerr({msg}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def loginfo(msg):
    res = None
    while res is None:
        try:
            res = rospy.loginfo(msg)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling loginfo({msg}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def logwarn(msg):
    res = None
    while res is None:
        try:
            res = rospy.logwarn(msg)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling logwarn({msg}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def init_node(name, *args, **kwargs):
    res = None
    while res is None:
        try:
            res = rospy.init_node(name, *args, **kwargs)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling init_node({name}, *{args}, **{kwargs}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def resolve_name(name):
    res = None
    while res is None:
        try:
            res = rospy.resolve_name(name)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling resolve_name({name}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def get_name():
    res = None
    while res is None:
        try:
            res =rospy.get_name()
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling get_name(). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res
        

def get_param_names():
    res = None
    while res is None:
        try:
            res =rospy.get_param_names()
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling get_param_names(). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def get_param(name, default=None):
    res = None
    while res is None:
        try:
            res =rospy.get_param(name, default)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling get_param({name},{default}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res
        
        
def set_param(name, value):
    res = None
    while res is None:
        try:
            res = rospy.set_param(name, value)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling set_param({name},{value}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res


def delete_param(name):
    res = None
    while res is None:
        try:
            res = rospy.delete_param(name)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr("Pyros : got socket error calling delete_param({name}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res
        
        
class MasterAPI_safe(object):
    def __init__(self, ms_proxy):
        self.ms_proxy = ms_proxy

    def lookupNode(self, caller_id):
        res = None
        while res is None:
            try:
                res = self.ms_proxy.lookupNode(caller_id)
            except (socket.error, socket.herror, socket.gaierror) as e:
                rospy.logerr("Pyros : got socket error calling MasterAPI_safe:lookupNode({caller_id}). Retrying...".format(
                    **locals()))
                time.sleep(retry_timeout)
            else:
                return res

    def getSystemState(self):
        res = None
        while res is None:
            try:
                res = self.ms_proxy.getSystemState()
            except (socket.error, socket.herror, socket.gaierror) as e:
                rospy.logerr("Pyros : got socket error calling MasterAPI_safe:getSystemState(). Retrying...".format(**locals()))
                time.sleep(retry_timeout)
            else:
                return res
    
    def getTopicTypes(self):
        res = None
        while res is None:
            try:
                res = self.ms_proxy.getTopicTypes()
            except (socket.error, socket.herror, socket.gaierror) as e:
                rospy.logerr("Pyros : got socket error calling MasterAPI_safe:getTopicTypes(). Retrying...".format(**locals()))
                time.sleep(retry_timeout)
            else:
                return res


def get_master():
    res = None
    while res is None:
        try:
            res = MasterAPI_safe(rospy.get_master())
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr(
                "Pyros : got socket error calling get_master(). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res

# Forwarding class definitions
Subscriber = rospy.Subscriber
Publisher = rospy.Publisher
Service = rospy.Service
ServiceProxy = rospy.ServiceProxy

rostime = rospy.rostime

__all__ = [
    'logerr',
    'loginfo',
    'logwarn',
    'resolve_name',
    'get_param',
    'set_param',
    'delete_param',
    'init_node',
    'get_name',
    'get_param_names',
    'get_master',
    'Publisher',
    'Subscriber',
    'Service',
    'ServiceProxy',
    'rostime',
]