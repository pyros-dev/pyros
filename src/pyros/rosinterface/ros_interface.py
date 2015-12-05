from __future__ import absolute_import

import rospy
import rosservice, rostopic, rosparam

import re
import ast
import socket

from pyros.mockinterface.baseinterface import BaseInterface

from .action import ActionBack
from .service import ServiceBack
from .topic import TopicBack
from .param import ParamBack

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


    def get_param_list(self):  # function returning all params available on the system
        return self.params_available

    def param_type_resolver(self, param_name):  # function resolving the type of a param
        return self.params_available_type.get(param_name, None)

    def ParamMaker(self, param_name, param_type):  # the param class implementation
        return ParamBack(param_name, param_type)

    def update(self):
        """
        Redefining update method in child class to gather all information in one master call per update.
        :return Return the difference tuple of services/topics/params exposed/withhold (NOT the detected changes we should not care about).
                Note that the name must match a regex previously set by the expose call
        """
        # TODO : improve this by using another node that caches master state
        try:
            _, _, system_state = self._master.getSystemState()
            publishers, subscribers, services = system_state

            # TODO : improvement : get the types here already ( no need to recall master everytime to get it )
            # Current status : getting all topic types is another master request and might not be always needed
            #  -> dont call again the master from here.

            self.topics_available = set([t[0] for t in (publishers + subscribers)])
            self.services_available = set([s[0] for s in services])

            self.params_available = set(rospy.get_param_names())

        except socket.error:
            rospy.logerr("[] couldn't get system state from the master ")

        return super(RosInterface, self).update()

    def reconfigure(self, config, level):
        """
        This callback is called when dynamic_reconfigure gets an update on
        parameter information. Topics which are received through here will be
        added to the list of topics which are monitored and added to or removed
        from the view on the REST interface
        """
        print(config)
        new_services = None
        new_topics = None
        new_params = None
        rospy.logwarn("""[{name}] Interface Reconfigure Request: \ntopics : {topics} \nservices : {services}""".format(name=__name__, **config))
        try:
            # convert new services to a set and then back to a list to ensure uniqueness
            new_services = list(set(ast.literal_eval(config["services"])))
        except ValueError:
            rospy.logwarn('[{name}] Ignored list {services} containing malformed service strings. Fix your input!'.format(name=__name__, **config))
        try:
            # convert new topics to a set and then back to a list to ensure uniqueness
            new_topics = list(set(ast.literal_eval(config["topics"])))
        except ValueError:
            rospy.logwarn('[{name}] Ignored list {topics} containing malformed topic strings. Fix your input!'.format(name=__name__, **config))
        try:
            # convert new params to a set and then back to a list to ensure uniqueness
            new_params = list(set(ast.literal_eval(config["params"])))
        except ValueError:
            rospy.logwarn('[{name}] Ignored list {params} containing malformed param strings. Fix your input!'.format(name=__name__, **config))

        self.reinit(new_services, new_topics, new_params)
        return config

BaseInterface.register(RosInterface)


