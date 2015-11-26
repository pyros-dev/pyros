from __future__ import absolute_import

import ast
import json
import os
import logging
import time

import zmp
# python protocol should be usable without ROS.
from ..pyros_prtcl import MsgBuild, Topic, Service, Param, ServiceList, ServiceInfo, TopicList, TopicInfo, ParamList, ParamInfo, Interaction, Interactions, InteractionInfo, Namespaces, NamespaceInfo, Rocon
from multiprocessing import Pipe
import threading


class PyrosMock(zmp.Node):
    """
    Mock Interface in pure python ( No ROS needed ).
    """
    # TODO : clean all this up now that we are using zmp.Node as parent
    def __init__(self, name='pyros-mock'):
        super(PyrosMock, self).__init__(name)
        self._topic_msg = {}  # storage for the echo topic
        self._param_val = {}  # storage for the test param
        self._stop_event = None  # stop_event to signal the thread for soft shutdown
        self._spinner = None  # thread instance
        self.provides(self.msg_build)
        self.provides(self.topic)
        self.provides(self.topic_list)
        self.provides(self.service)
        self.provides(self.service_list)
        self.provides(self.param)
        self.provides(self.param_list)
        self.provides(self.interactions)
        self.provides(self.namespaces)
        self.provides(self.interaction)
        self.provides(self.has_rocon)
        pass

    # These should match the design of PyrosClient and Protocol so we are consistent between pipe and python API
    def msg_build(self, name):
        # TODO : better mock : this can except if not string.
        msg = str()
        return msg

    # a simple echo topic
    def topic(self, name, msg_content=None):
        msg = msg_content
        if msg_content is not None:
            self._topic_msg[name] = msg_content
            msg = None  # consuming the message
        else:
            msg = self._topic_msg.get(name, None)
        return msg
        
    def topic_list(self):
        info = TopicInfo(fullname='mock fullname', allow_sub=False)
        resp_content = {'mock': info}
        return resp_content

    # a simple string echo service
    def service(self, name, rqst_content=None):
        # simulating a strict message typing backend
        #if not isinstance(rqst_content, str):
        #    raise Exception("Request Content Not Expected Type !")
        resp_content = rqst_content
        return resp_content

    def service_list(self):
        info = ServiceInfo(fullname='mock_fullname')
        resp_content = {'mock': info}
        return resp_content

    # a simple test param
    def param(self, name, value=None):
        val = value
        if value is not None:
            self._param_val[name] = value
            val = None  # consuming the message
        else:
            val = self._param_val.get(name, None)
        return val

    def param_list(self):
        info = ParamInfo(fullname='mock fullname')
        resp_content = {'mock': info}
        return resp_content

    def namespaces(self):
        info = NamespaceInfo()
        resp_content = {}
        return resp_content

    def interaction(self, name):
        resp_content = Interaction(interaction=name)
        return resp_content

    def interactions(self):
        resp_content = {}
        return resp_content

    def has_rocon(self):
        return False

    # def _dispatch_msg(self, pipe_conn):
    #     rqst = pipe_conn.recv()
    #     resp = rqst  # if problem we send back the exact same request message.
    #     # here we need to make sure we always send something back ( so the client can block safely )
    #     try:
    #         if isinstance(rqst, MsgBuild):
    #             resp = MsgBuild(
    #                 name=rqst.name,
    #                 msg_content=self.msg_build(rqst.name)
    #             )
    #         elif isinstance(rqst, Topic):
    #             resp = Topic(
    #                 name=rqst.name,
    #                 msg_content=self.topic(rqst.name, rqst.msg_content)
    #             )
    #         elif isinstance(rqst, TopicList):
    #             resp = TopicList(
    #                 name_dict=self.topic_list()
    #             )
    #         elif isinstance(rqst, Service):
    #             resp = Service(
    #                 name=rqst.name,
    #                 rqst_content=rqst.rqst_content,
    #                 resp_content=self.service(rqst.name, rqst.rqst_content)
    #             )
    #         elif isinstance(rqst, ServiceList):
    #             resp = ServiceList(
    #                 name_dict=self.service_list()
    #             )
    #         elif isinstance(rqst, Param):
    #             resp = Param(
    #                 name=rqst.name,
    #                 value=self.param(rqst.name, rqst.value)
    #             )
    #         elif isinstance(rqst, ParamList):
    #             resp = ParamList(
    #                 name_dict=self.param_list()
    #             )
    #         elif isinstance(rqst, Interactions):
    #             resp = Interactions(
    #                 interaction_dict=self.interactions()
    #             )
    #         elif isinstance(rqst, Namespaces):
    #             resp = Namespaces(
    #                 namespace_dict=self.namespaces()
    #             )
    #         elif isinstance(rqst, Rocon):
    #             resp = Rocon(
    #                 has_rocon=self.has_rocon()
    #             )
    #         elif isinstance(rqst, Interaction):
    #             resp = Interaction(
    #                 interaction=self.interaction(rqst.name)
    #             )
    #     finally:
    #         # to make sure we always return something, no matter what
    #         pipe_conn.send(resp)

    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """

        logging.debug("mock running, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

        super(PyrosMock, self).run()

        logging.debug("mock shutdown, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))



