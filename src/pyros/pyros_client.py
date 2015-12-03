from __future__ import absolute_import

import logging
import unicodedata

"""
Client to rostful node, Python style.
Required for multiprocess communication.
"""

import zmp
from .pyros_prtcl import MsgBuild, Topic, Service, Param, ParamList, ParamInfo, ServiceList, ServiceInfo, TopicList, TopicInfo, Namespaces, NamespaceInfo, Interactions, InteractionInfo, Rocon


class PyrosClient(object):
    def __init__(self, node_name = None):
        # Discover all Services. Block if one not there
        # TODO : timeout exception ?
        self.msg_build_svc = zmp.Service.discover('msg_build', node_name)
        self.topic_svc = zmp.Service.discover('topic', node_name)
        self.service_svc = zmp.Service.discover('service', node_name)
        self.param_svc = zmp.Service.discover('param', node_name)
        self.topic_list_svc = zmp.Service.discover('topic_list', node_name)
        self.service_list_svc = zmp.Service.discover('service_list', node_name)
        self.param_list_svc = zmp.Service.discover('param_list', node_name)

        self.namespaces_svc = zmp.Service.discover('namespaces', node_name)
        self.interactions_svc = zmp.Service.discover('interactions', node_name)
        self.interaction_svc = zmp.Service.discover('interaction', node_name)
        self.has_rocon_svc = zmp.Service.discover('has_rocon', node_name)

    def buildMsg(self, connection_name, suffix=None):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(connection_name, unicode):
            connection_name = unicodedata.normalize('NFKD', connection_name).encode('ascii', 'ignore')
        res = self.msg_build_svc.call(args=(connection_name,))
        return res

    def topic_inject(self, topic_name, _msg_content={}, **kwargs):
        """
        Injecting message into topic. is _msg_content, we inject it directly. if not, we use all extra kwargs
        :param topic_name: name of the topic
        :param _msg_content: optional message content
        :param kwargs: each extra kwarg will be put int he message is structure matches
        :return:
        """
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        if kwargs:
            res = self.topic_svc.call(args=(topic_name, kwargs,))
        elif _msg_content is not None:
            res = self.topic_svc.call(args=(topic_name, _msg_content,))
        else:   # if _msg_content is None the request is invalid.
                # just return something to mean False.
            res = 'WRONG INJECT'

        return res is None  # check if message has been consumed

    def topic_extract(self, topic_name):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        res = self.topic_svc.call(args=(topic_name, None,))

        return res

    def service_call(self, service_name, _msg_content={}, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(service_name, unicode):
            service_name = unicodedata.normalize('NFKD', service_name).encode('ascii', 'ignore')

        if kwargs:
            res = self.service_svc.call(args=(service_name, kwargs,))
        elif _msg_content is not None:
            res = self.service_svc.call(args=(service_name, _msg_content,))
        else:   # if _msg_content is None the request is invalid.
                # just return None.
            res = None


        # A service that doesn't exist on the node will return res_content.resp_content None.
        # It should probably except...
        # TODO : improve error handling, maybe by checking the type of res ?

        return res

    def param_set(self, param_name, _value={}, **kwargs):
        """
        Setting parameter. if _value, we inject it directly. if not, we use all extra kwargs
        :param topic_name: name of the topic
        :param _value: optional value
        :param kwargs: each extra kwarg will be put in the value if structure matches
        :return:
        """
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(param_name, unicode):
            param_name = unicodedata.normalize('NFKD', param_name).encode('ascii', 'ignore')

        if kwargs:
            res = self.param_svc.call(args=(param_name, kwargs,))
        elif _value is not None:
            res = self.param_svc.call(args=(param_name, _value,))
        else:   # if _msg_content is None the request is invalid.
                # just return something to mean False.
            res = 'WRONG SET'

        return res is None  # check if message has been consumed

    def param_get(self, param_name):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(param_name, unicode):
            param_name = unicodedata.normalize('NFKD', param_name).encode('ascii', 'ignore')
        res = self.param_svc.call(args=(param_name, None,))
        return res

    def listtopics(self):
        res = self.topic_list_svc.call(args=({},))
        return res
        
    def listsrvs(self):
        res = self.service_list_svc.call(args=({},))
        return res

    def listparams(self):
        res = self.param_list_svc.call(args=({},))
        return res

    def listacts(self):
        return {}

    def namespaces(self):
        res = self.namespaces_svc.call(args=({},))
        return res

    def interactions(self):
        res = self.interactions_svc.call(args=({},))
        return res

    def interaction(self, name):
        res = self.interaction_svc.call(args=("",))
        return res

    def has_rocon(self):
        res = self.interaction_svc.call(args=(False,))
        return res

# TODO : test client with Rostful Node ( and detect ROS or not to confirm behvior )
