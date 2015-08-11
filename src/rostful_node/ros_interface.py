from __future__ import absolute_import

import roslib
import rospy
from rospy.service import ServiceManager
import rosservice, rostopic
import actionlib_msgs.msg
import string

from importlib import import_module
from collections import deque

import json
import sys
import re
from StringIO import StringIO

from rosinterface import message_conversion as msgconv
from rosinterface import definitions, util
from rosinterface.util import ROS_MSG_MIMETYPE, request_wants_ros, get_query_bool

import os
import urlparse
import ast

from rosinterface import ActionBack
from rosinterface import ServiceBack
from rosinterface import TopicBack

from .ros_watcher import ROSWatcher

import unicodedata

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'
ACTION_PATH = '_action'

def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH,SRV_PATH,MSG_PATH,ACTION_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

def response(start_response, status, data, content_type):
    content_length = 0
    if data is not None:
        content_length = len(data)
    headers = [('Content-Type', content_type), ('Content-Length', str(content_length))]
    start_response(status, headers)
    return data
#TODO clean this
def response_200(start_response, data='', content_type='application/json'):
    return response(start_response, '200 OK', data, content_type)

def response_404(start_response, data='Invalid URL!', content_type='text/plain'):
    return response(start_response, '404 Not Found', data, content_type)

def response_405(start_response, data=[], content_type='text/plain'):
    return response(start_response, '405 Method Not Allowed', data, content_type)

def response_500(start_response, error, content_type='text/plain'):
    e_str = '%s: %s' % (str(type(error)), str(error))
    return response(start_response, '500 Internal Server Error', e_str, content_type)

class ActionNotExposed(Exception):
    def __init__(self, action_name):
        self.action_name = action_name
    pass


"""
Interface with ROS.
"""
class RosInterface(object):

    def __init__(self, run_watcher=True):
        # Current services topics and actions exposed, i.e. those which are
        # active in the system.
        self.services = {}
        self.topics = {}
        self.actions = {}
        # The *_waiting and *args should be consistent with the initial
        # character of the strings they contain. Currently we ensure that all
        # the lists contain strings which do not have a leading slash.
        
        # Current services topics and actions we are waiting for. These are
        # either those which have not yet appeared, or which have disappeared
        # from the system.
        self.services_waiting = []
        self.topics_waiting = []
        self.actions_waiting = []
        # Last requested services topics and actions to be exposed, received
        # from a reconfigure request. Topics which match topics containing
        # wildcards go in here after they are added, but when a new reconfigure
        # request is received, they disappear. The value of the topic and action
        # dicts is the number of instances that that that item has, i.e. how
        # many times the add function has been called for the given key.
        self.services_args = []
        self.topics_args = {} # dict to keep track of the number of connections
        self.actions_args = {}
        #current topics waiting for deletion ( still contain messages )
        self.topics_waiting_del = {}

        if run_watcher:
            self.ros_watcher = ROSWatcher(self.topics_change_cb, self.services_change_cb, self.actions_change_cb)
            self.ros_watcher.start()

    ##
    # Attach beginning of line and end of line characters to the given string.
    # Ensures that raw topics like /test do not match other topics containing
    # the same string (e.g. /items/test would result in a regex match at char 7)
    # regex goes through each position in the string to find matches - this
    # forces it to consider only the first position
    def cap_match_string(self, match):
        return '^' + match + '$'
        
    ##
    # @param key The topic, action or service name to check against the strings
    # that we have in the list of matchable candidates
    # @param match_candidates list of match candidates that we should try to match against
    def is_regex_match(self, key, match_candidates):
        for cand in match_candidates:
            try:
                pattern = re.compile(self.cap_match_string(cand))
                if pattern.match(key):
                    return True
            except:
                rospy.logwarn('[ros_interface] Invalid regex string "%s"!' % cand)

        return False

    ##
    # This callback is called when dynamic_reconfigure gets an update on
    # parameter information. Topics which are received through here will be
    # added to the list of topics which are monitored and added to or removed
    # from the view on the REST interface
    def reconfigure(self, config, level):
        print(config)
        rospy.logwarn("""[ros_interface] Interface Reconfigure Request: \ntopics : {topics} \nservices : {services} \nactions : {actions}""".format(**config))
        try:
            # convert new topics to a set and then back to a list to ensure uniqueness
            new_topics = list(set(ast.literal_eval(config["topics"])))
            self.expose_topics(new_topics)
        except ValueError:
            rospy.logwarn('[ros_interface] Ignored list %s containing malformed topic strings. Fix your input!' % str(config["topics"]))
        try:
            # convert new services to a set and then back to a list to ensure uniqueness
            new_services = list(set(ast.literal_eval(config["services"])))
            self.expose_services(new_services)
        except ValueError:
            rospy.logwarn('[ros_interface] Ignored list %s containing malformed service strings. Fix your input!' % str(config["services"]))
            
        try:
            # convert new actions to a set and then back to a list to ensure uniqueness
            new_actions = list(set(ast.literal_eval(config["actions"])))
            self.expose_actions(new_actions)
        except ValueError:
            rospy.logwarn('[ros_interface] Ignored list %s containing malformed action strings. Fix your input!' % str(config["actions"]))

        return config

    def add_service(self, service_name, ws_name=None, service_type=None):
        rospy.loginfo("[ros_interface] Adding service %s" % service_name)
        resolved_service_name = rospy.resolve_name(service_name)
        if service_name not in self.services_args:
            self.services_args.append(service_name)
            
        if service_type is None:
            try:
                service_type = rosservice.get_service_type(resolved_service_name)
                if not service_type:
                    rospy.logwarn('[ros_interface] Cannot Expose unknown service %s (maybe it is a regex or doesn\'t exist yet?)' % service_name)
                    self.services_waiting.append(service_name)
                    return False
            except rosservice.ROSServiceIOException, e:
               rospy.logwarn('[ros_interface] Error trying to Expose service {name} : {error}'.format(name=service_name, error=e))
               self.services_waiting.append(service_name)
               return False

        # only create a new backend for the service when it is in the waiting list
        if service_name in self.services_waiting:
            self.services_waiting.remove(service_name)
        
        self.services[service_name] = ServiceBack(service_name, service_type)
        return True

    ##
    # @return false if the service did not exist, true if the connection was
    # deleted.
    def del_service(self, service_name, force=False):
        rospy.loginfo("[ros_interface] Deleting service %s" % service_name)
        if service_name in self.services:
            if force:
                self.services_args.remove(service_name)
                if service_name in self.services_waiting:
                    self.services_waiting.remove(service_name)
            else:
                self.services_waiting.append(service_name)
                
            self.services.pop(service_name, None)
            return True
        return False

    """
    This exposes a list of services as REST API. services not listed here will be removed from the API
    """
    def expose_services(self, service_names):
        rospy.loginfo('[ros_interface] Exposing services : %r', service_names)

        if not service_names:
            return

        # look through the new service names received by reconfigure, and add
        # those services which are not in the existing service args
        for service_name in service_names:
            if not service_name in self.services_args:
                ret = self.add_service(service_name)

        # look through the current service args and delete those values which
        # will not be valid when the args are replaced with the new ones. run on
        # a copy so that we will remove from the original without crashing
        for service_name in list(self.services_args):
            if not service_name in service_names or not self.is_regex_match(service_name, service_names):
                ret = self.del_service(service_name, force=True)

        self.services_args = service_names

    def get_service(self, service_name):
        if service_name in self.services.keys():
            service = self.services[service_name]
            return service
        else:
            return None  # service not exposed

    def add_topic(self, topic_name, topic_type=None, allow_pub=True, allow_sub=True):
        resolved_topic_name = rospy.resolve_name(topic_name)
        # this initialises regex topics
        if topic_name not in self.topics_args:
            self.topics_args[topic_name] = -2
        if self.topics_args[topic_name] >= 0:
            rospy.loginfo("[ros_interface] Adding topic %s" % topic_name)

        if topic_type is None:
            try:
                topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
                if not topic_type:
                    rospy.logwarn('[ros_interface] Cannot Expose unknown topic %s (maybe it is a regex or doesn\'t exist yet?)' % topic_name)
                    self.topics_waiting.append(topic_name)
                    return False
            except rosservice.ROSServiceIOException, e:
                rospy.logwarn('[ros_interface] Error trying to Expose topic {name} : {error}'.format(name=topic_name, error=e))
                self.topics_waiting.append(topic_name)
                return False
            
        if topic_name in self.topics_waiting_del.keys() > 0:
            # here the intent is obviously to erase the old homonym topic data
            self.topics_waiting_del.pop(topic_name, None)

        # if this is a regex match topic, it will not be in the waiting list
        # just after it is initialised above
        if topic_name in self.topics_waiting:
            self.topics_waiting.remove(topic_name)
        # if the topic didn't exist yet, create a backend
        if self.topics_args[topic_name] == -2:
            self.topics[topic_name] = TopicBack(topic_name, topic_type, allow_pub=allow_pub, allow_sub=allow_sub)

        self.topics_args[topic_name] += 1
        
        return True

    ##
    # @param force force deletion of the topic - remove it from the list of
    # topics, waiting list and args, ignoring the total number of connections to
    # the topic, and whether or not it has waiting messages
    # @return false if the topic did not exist, or there was more than one
    # connection to the topic, true if the last connection was deleted.
    def del_topic(self, topic_name, noloss=False, force=False):
        # can only delete topic if we have it in the topic list
        if topic_name in self.topics:
            rospy.loginfo("[ros_interface] Deleting topic %s" % topic_name)
            if force:
                self.topics_args.pop(topic_name)
                self.topics.pop(topic_name)
                if topic_name in self.topics_waiting:
                    self.topics_waiting.remove(topic_name)
                return True
                
            # if there is more than one connection to the topic, we decrement
            # the count
            if self.topics_args[topic_name] > 1:
                self.topics_args[topic_name] -= 1
                return False

            # if there is only one connection left, and the topic is a noloss
            # topic with more than one message left, add it to the list of
            # topics waiting for deletion
            if noloss and self.topics.get(topic_name).unread() > 0 and self.topics_args[topic_name] == 1:
                # we want to delete it later, after last message has been consumed
                # we make a copy of the topic to still be able to access it
                self.topics_waiting_del[topic_name] = (self.topics.get(topic_name))
            elif not noloss and topic_name in self.topics_waiting_del:  # in this case we want to actually remove it completely
                self.topics_waiting_del.pop(topic_name, None)

            # if there is only one connection left, remove the topic from the
            # list, add it to the waiting list, and set its number of
            # connections to zero.
            if self.topics_args[topic_name] == 1:
                self.topics.pop(topic_name, None)
                self.topics_waiting.append(topic_name)
                self.topics_args[topic_name] = -2
        else:
            return False
        return True

    """
    This exposes a list of topics as REST API. topics not listed here will be removed from the API
    """
    def expose_topics(self, topic_names, allow_pub=True, allow_sub=True):
        rospy.loginfo('[ros_interface] Exposing topics : %r', topic_names)
        if not topic_names:
            return

        # look through the new topic names received by reconfigure, and add
        # those services which are not in the existing topic args
        for topic_name in topic_names:
            if not topic_name in self.topics_args:
                ret = self.add_topic(topic_name, allow_pub=allow_pub, allow_sub=allow_sub)

        # look through the current topic args and delete those values which will
        # not be valid when the args are replaced with the new ones. use
        # .items() copy to allow removal of items from the dict in the del_topic
        # function
        for topic_name in self.topics_args.items():
            if not topic_name[0] in topic_names or not self.is_regex_match(topic_name[0], topic_names):
                rospy.loginfo("topic %s not in topic names %r" % (topic_name[0], topic_names))
                ret = self.del_topic(topic_name[0], force=True)

    def get_topic(self, topic_name):
        #normalizing names... ( somewhere else ?)
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        #hiding topics waiting for deletion with no messages waiting to be read.
        if topic_name in self.topics.keys():
            topic = self.topics[topic_name]
            return topic
        elif topic_name in self.topics_waiting_del.keys() and self.topics_waiting_del[topic_name].unread() > 0:
            topic = self.topics_waiting_del[topic_name]
            return topic
        else:
            return None  # topic not exposed

    def add_action(self, action_name, action_type=None):
        rospy.loginfo("[ros_interface] Adding action %s" % action_name)
        if action_name not in self.actions_args:
            self.action_args[action_name] = 0
        if action_type is None:
            resolved_topic_name = rospy.resolve_name(action_name + '/result')
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if not topic_type:
                rospy.logwarn( 'Cannot Expose unknown action %s (maybe it is a regex, or doesn\'t exist yet?)', action_name )
                self.actions_waiting.append(action_name)
                return False
            action_type = topic_type[:-len('ActionResult')]

        if action_name in self.actions_waiting or self.actions_args[action_name] == 0:
            self.actions_waiting.remove(action_name)
            self.actions[action_name] = ActionBack(action_name, action_type)
            
        self.actions_args[action_name] += 1
        return True

    ##
    # @return false if the action did not exist, or there was more than one
    # connection to the action, true if the last connection was deleted.
    def del_action(self, action_name):
        if action_name in self.actions:
            rospy.loginfo("Deleting action %s" % action_name)
            if self.actions_args[action_name] == 1:
                self.actions.pop(action_name, None)
                self.actions_waiting.append(action_name)
                self.actions_args[action_name] = 0
                return True
            else:
                self.actions_args[action_name] -= 1

        return False

    """
    This exposes a list of actions as REST API. actions not listed here will be removed from the API
    """
    def expose_actions(self, action_names):
        rospy.loginfo('[ros_interface] Exposing actions : %r', action_names)
        if not action_names:
            return
        for action_name in action_names:
            if not action_name in self.actions_args:
                ret = self.add_action(action_name)
                #if ret: rospy.loginfo( 'Exposed Action %s', action_name)

        for action_name in self.actions_args.items():
            if not action_name[0] in action_names or not self.is_regex_match(action_name[0], action_names):
                ret = self.del_action(action_name[0])
                #if ret: rospy.loginfo ( 'Removed Action %s', action_name)

        # Updating the list of actions


    def get_action(self, action_name):
        if action_name in self.actions:
            action = self.actions[action_name]
            return action
        else:
            raise ActionNotExposed(action_name)
            
    ##
    # This callback is called when the ros_watcher receives information about
    # new topics, or topics which dropped off the ros network.
    def topics_change_cb(self, new_topics, lost_topics):
        # internal lists store topics without the initial slash, but we receive them with it from outside
        new_topics = new_topics
        lost_topics = lost_topics
        # rospy.logwarn('new topics : %r, lost topics : %r' % (new_topics, lost_topics))
        # convert new topics to a set and then back to a list to ensure uniqueness
        topics_lst = [t for t in list(set(new_topics)) if t in self.topics_args or self.is_regex_match(t, self.topics_waiting)]
        if len(topics_lst) > 0:
            # rospy.logwarn('exposing new topics : %r', topics_lst)
            # Adding missing ones
            for topic_name in topics_lst:
                ret = self.add_topic(topic_name)
        # rospy.loginfo("Currently waiting on topics %r" % self.topics_waiting)
        # rospy.loginfo("current topics %r" % self.topics)
                
        topics_lst = [t for t in lost_topics if t in self.topics_args]
        if len(topics_lst) > 0:
            # rospy.logwarn('hiding lost topics : %r', topics_lst)
            # Removing extra ones
            for topic_name in topics_lst:
                ret = self.del_topic(topic_name)

        # taking the opportunity to try cleaning the topics that have been emptied
        # TODO : think about a clean way to link that to the topic.get() method
        if len(self.topics_waiting_del) > 0:
            cleanup = []
            for ws_name in self.topics_waiting_del.keys():
                if 0 == self.topics_waiting_del.get(ws_name).unread():  # FIXME : careful about the difference between topic_name and ws_name
                    cleanup.append(ws_name)
            for ws_name in cleanup:
                self.topics_waiting_del.pop(ws_name, None)
                #TODO : cleaner way by calling self.del_topic ?

    def services_change_cb(self, new_services, lost_services):
        # internal lists store topics without the initial slash, but we receive them with it from outside
        new_services = new_services
        lost_services = lost_services
        # rospy.logwarn('new services : %r, lost services : %r' % (new_services, lost_services))
        # convert new services to a set and then back to a list to ensure uniqueness
        svc_list = [s for s in list(set(new_services)) if s in self.services_args or self.is_regex_match(s, self.services_waiting)]
        if len(svc_list) > 0:
            # rospy.logwarn('exposing new services : %r', svc_list)
            for svc_name in svc_list:
                self.add_service(svc_name)
        # rospy.loginfo("Currently waiting on services %r" % self.services_waiting)
        # rospy.loginfo("current services %r" % self.services)

        svc_list = [s for s in lost_services if s in self.services_args]
        if len(svc_list) > 0:
            # rospy.logwarn('hiding lost services : %r', svc_list)
            for svc_name in svc_list:
                self.del_service(svc_name)

    def actions_change_cb(self, new_actions, lost_actions):
        # internal lists store topics without the initial slash, but we receive them with it from outside
        new_actions = new_actions
        lost_actions = lost_actions
        # rospy.logwarn('new actions : %r, lost actions : %r', new_actions, lost_actions)
        # convert new actions to a set and then back to a list to ensure uniqueness
        act_list = [a for a in list(set(new_actions)) if a in self.actions_args or self.is_regex_match(a, self.topics_waiting)]
        if len(act_list) > 0:
            # rospy.logwarn('exposing new actions : %r', act_list)
            for act_name in act_list:
                self.add_action(act_name)

        act_list = [a for a in lost_actions if a in self.actions_args]
        if len(act_list) > 0:
            # rospy.logwarn('hiding lost actions : %r', act_list)
            for act_name in act_list:
                self.del_action(act_name)

