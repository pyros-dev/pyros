from __future__ import absolute_import

import logging
import unicodedata

"""
Client to rostful node, Python style.
Required for multiprocess communication.
"""

from .rostful_prtcl import MsgBuild, Topic, Service, ServiceList, ServiceInfo, TopicList, TopicInfo, Namespaces, NamespaceInfo, Interactions, InteractionInfo, Rocon

class RostfulClient(object):
    def __init__(self, pipe_conn):
        self._pipe_conn = pipe_conn

    def buildMsg(self, connection_name, suffix=None):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(connection_name, unicode):
            connection_name = unicodedata.normalize('NFKD', connection_name).encode('ascii', 'ignore')
        try:
            self._pipe_conn.send(MsgBuild(name=connection_name, msg_content=None))
            res = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_content if isinstance(res, MsgBuild) else None

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

        try:
            if kwargs:
                self._pipe_conn.send(Topic(name=topic_name, msg_content=kwargs))
                res = self._pipe_conn.recv()
            elif _msg_content is not None:
                self._pipe_conn.send(Topic(name=topic_name, msg_content=_msg_content))
                res = self._pipe_conn.recv()
            else:   # if _msg_content is None the request is invalid.
                    # Don't send anything into the pipe and just return False.
                res = Topic(name=topic_name, msg_content='NONE IS NOT CONSUMED')

        except Exception, e:
            raise

        return res.msg_content is None  # check if message has been consumed

    def topic_extract(self, topic_name):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        try:
            self._pipe_conn.send(Topic(name=topic_name, msg_content=None))
            res = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_content

    def service_call(self, service_name, _msg_content={}, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(service_name, unicode):
            service_name = unicodedata.normalize('NFKD', service_name).encode('ascii', 'ignore')

        try:
            if kwargs:
                self._pipe_conn.send(Service(name=service_name, rqst_content=kwargs, resp_content=None))
                res_content = self._pipe_conn.recv()
            elif _msg_content is not None:
                self._pipe_conn.send(Service(name=service_name, rqst_content=_msg_content, resp_content=None))
                res_content = self._pipe_conn.recv()
            else:   # if _msg_content is None the requeset is invalid.
                    # Dont send anything into the pipe and just return None.
                res_content = Service(name=service_name, rqst_content=None, resp_content=None)

        except Exception, e:
            raise

        # A service that doesn't exist on the node will return res_content.resp_content None.
        # It should probably except...
        # TODO : improve error handling, maybe by checking the type of res_content ?

        return res_content.resp_content

    def listtopics(self):
        try:
            self._pipe_conn.send(TopicList(name_dict={}))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res_content.name_dict
        
    def listsrvs(self):
        try:
            self._pipe_conn.send(ServiceList(name_dict={}))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise
            
        return res_content.name_dict

    def listacts(self):
        return {}

    def namespaces(self):
        try:
            self._pipe_conn.send(Namespaces(namespace_dict={}))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res_content.namespace_dict

    def interactions(self):
        try:
            self._pipe_conn.send(Interactions(interaction_dict={}))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise
            
        return res_content.interaction_dict

    def interaction(self, name):
        try:
            self._pipe_conn.send(Interaction(interaction=""))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res_content.interaction

    def has_rocon(self):
        try:
            self._pipe_conn.send(Rocon(has_rocon=False))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res_content.has_rocon

# TODO : test client with Rostful Node ( and detect ROS or not to confirm behvior )
