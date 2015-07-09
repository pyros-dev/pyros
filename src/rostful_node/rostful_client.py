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
        self.pipe_conn = pipe_conn

    def inject(self, topic_name, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        try:
            self.pipe_conn.send(Topic(name=topic_name, msg_content=kwargs))
            res = self.pipe_conn.recv()
        except Exception, e:
            raise

        return res.msg_content is None  # check if message has been consumed

    def extract(self, topic_name):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(topic_name, unicode):
            topic_name = unicodedata.normalize('NFKD', topic_name).encode('ascii', 'ignore')

        try:
            self.pipe_conn.send(Topic(name=topic_name, msg_content=None))
            res = self.pipe_conn.recv()
        except Exception, e:
            raise

        return res

    def call(self, service_name, **kwargs):
        #changing unicode to string ( testing stability of multiprocess debugging )
        if isinstance(service_name, unicode):
            service_name = unicodedata.normalize('NFKD', service_name).encode('ascii', 'ignore')

        try:
            self.pipe_conn.send(Service(name=service_name, rqst_content=kwargs, resp_value=None))
            res_content = self.pipe_conn.recv()
        except Exception, e:
            raise

        return res_content

