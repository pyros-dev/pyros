from __future__ import absolute_import

import logging
import unicodedata

"""
Client to rostful node, Python style.
Required for multiprocess communication.
"""

from .rostful_prtcl import MsgBuild, Topic, Service

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

    def inject(self, topic_name, _msg_content=None, **kwargs):
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
            if _msg_content:
                self._pipe_conn.send(Topic(name=topic_name, msg_content=_msg_content))
            elif kwargs:
                self._pipe_conn.send(Topic(name=topic_name, msg_content=kwargs))
            else:
                self._pipe_conn.send(Topic(name=topic_name, msg_content=None))
            res = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_content is None  # check if message has been consumed

    def extract(self, topic_name):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        try:
            self._pipe_conn.send(Topic(name=topic_name, msg_content=None))
            res = self._pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_content

    def call(self, service_name, _msg_content=None, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(service_name, unicode):
            service_name = unicodedata.normalize('NFKD', service_name).encode('ascii', 'ignore')

        try:
            if _msg_content:
                self._pipe_conn.send(Service(name=service_name, rqst_content=_msg_content, resp_content=None))
            elif kwargs:
                self._pipe_conn.send(Service(name=service_name, rqst_content=kwargs, resp_content=None))
            else:  # should we always pass {} if no kwargs ?
                self._pipe_conn.send(Service(name=service_name, rqst_content=None, resp_content=None))
            res_content = self._pipe_conn.recv()
        except Exception, e:
            raise

        # A service that doesn't exist on the node will return res_content.resp_content None. It should probably except...
        # TODO : improve error handling, maybe by checking the type of res_content ?

        return res_content.resp_content

# TODO : test client with Rostful Node ( and detect ROS or not to confirm behvior )
