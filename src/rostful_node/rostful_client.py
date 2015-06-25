from __future__ import absolute_import

import logging
import unicodedata

"""
Client to rostful node, Python style.
Required for multiprocess communication.
"""

from rosinterface import message_conversion as msgconv
from .rostful_prtcl import MsgBuild, Topic, Service

class RostfulClient(object):
    def __init__(self, pipe_conn):
        self.pipe_conn = pipe_conn

    def buildMsg(self, connection_name, suffix=None):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(connection_name, unicode):
            connection_name = unicodedata.normalize('NFKD', connection_name).encode('ascii', 'ignore')
        try:
            self.pipe_conn.send(MsgBuild(connec_name=connection_name, msg_value=None))
            res = self.pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_value if isinstance(res, MsgBuild) else None

    def inject(self, topic_name, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        msg = self.buildMsg(topic_name)
        # build msg from kwargs ( smaller size than passing kwargs )
        # TODO check if we shouldn't pass kwargs instead for consistency for design
        msgconv.populate_instance(kwargs, msg)

        try:
            self.pipe_conn.send(Topic(name=topic_name, msg_value=msg))
            res = self.pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_value is None  # check if message has been consumed

    def extract(self, topic_name):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        try:
            self.pipe_conn.send(Topic(name=topic_name, msg_value=None))
            res = self.pipe_conn.recv()
        except Exception, e:
            raise

        res_obj = msgconv.extract_values(res.msg_value)

        return res_obj

    def call(self, service_name, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(service_name, unicode):
            service_name = unicodedata.normalize('NFKD', service_name).encode('ascii', 'ignore')

        rqst = self.buildMsg(service_name)
        # build msg from kwargs ( smaller size than passing kwargs )
        # TODO check if we shouldn't pass kwargs instead for consistency for design
        msgconv.populate_instance(kwargs, rqst)

        try:
            self.pipe_conn.send(Service(name=service_name, rqst_value=rqst, resp_value=None))
            res = self.pipe_conn.recv()
        except Exception, e:
            raise

        res_obj = msgconv.extract_values(res.resp_value)

        return res_obj

