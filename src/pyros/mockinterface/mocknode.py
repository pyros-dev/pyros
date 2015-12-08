from __future__ import absolute_import

import ast
import json
import os
import logging
import time

import zmp
# python protocol should be usable without ROS.
from pyros.baseinterface import PyrosBase
from ..pyros_prtcl import MsgBuild, Topic, Service, Param, ServiceList, ServiceInfo, TopicList, TopicInfo, ParamList, ParamInfo, Interaction, Interactions, InteractionInfo, Namespaces, NamespaceInfo, Rocon
from .mockinterface import MockInterface


# TODO : service interface for mock to be able to dynamically trigger services / topics / params appearing & disappearing
class PyrosMock(PyrosBase):
    """
    Mock Interface in pure python ( No ROS needed ).

    """
    def __init__(self, name='pyros-mock'):
        self.mock_if = MockInterface()
        # TODO : make the interface part of the super init. Needs to allow choosing hte interface subclass...

        super(PyrosMock, self).__init__(name)
        self._topic_msg = {}  # storage for the echo topic
        self._param_val = {}  # storage for the test param
        self._stop_event = None  # stop_event to signal the thread for soft shutdown
        self._spinner = None  # thread instance

        # Mock service to be able to trigger services / topics / params creation and temrination
        self.provides(self.mock_service_appear)
        self.provides(self.mock_service_disappear)
        self.provides(self.mock_topic_appear)
        self.provides(self.mock_topic_disappear)
        self.provides(self.mock_param_appear)
        self.provides(self.mock_param_disappear)
        pass

    # These should match the design of PyrosClient and Protocol so we are consistent between pipe and python API
    def msg_build(self, name):
        # TODO : better mock : this can except if not string.
        msg = str()
        return msg

    # a simple echo topic
    def topic(self, name, msg_content=None):
        # TODO : use Mock interface topics directly
        msg = msg_content
        if msg_content is not None:
            self._topic_msg[name] = msg_content
            msg = None  # consuming the message
        else:
            msg = self._topic_msg.get(name, None)
        return msg
        
    def topics(self):
        return self.mock_if.get_topic_list()

    # a simple string echo service
    def service(self, name, rqst_content=None):
        # simulating a strict message typing backend
        #if not isinstance(rqst_content, str):
        #    raise Exception("Request Content Not Expected Type !")
        resp_content = rqst_content
        return resp_content

    def services(self):
        return self.mock_if.get_svc_list()

    # a simple test param
    def param(self, name, value=None):
        val = value
        if value is not None:
            self._param_val[name] = value
            val = None  # consuming the message
        else:
            val = self._param_val.get(name, None)
        return val

    def params(self):
        return self.mock_if.get_param_list()

    def reinit(self, services, topics, params):
        """
        Allowing dynamic configuration of the process while it s running
        :param services:
        :param topics:
        :param params:
        :return:
        """
        return self.mock_if.reinit(services, topics, params)


    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """

        logging.debug("mock running, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

        super(PyrosMock, self).run()

        logging.debug("mock shutdown, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

    def update(self):
        """
        Update function to call from a looping thread.
        """

        self.mock_if.update()

    # Mock only methods
    def mock_service_appear(self, svc_name, svc_type):
        return self.mock_if.mock_service_appear(svc_name, svc_type)

    def mock_service_disappear(self, svc_name):
        return self.mock_if.mock_service_disappear(svc_name)

    def mock_topic_appear(self, svc_name, svc_type):
        return self.mock_if.mock_topic_appear(svc_name, svc_type)

    def mock_topic_disappear(self, svc_name):
        return self.mock_if.mock_topic_disappear(svc_name)

    def mock_param_appear(self, svc_name, svc_type):
        return self.mock_if.mock_param_appear(svc_name, svc_type)

    def mock_param_disappear(self, svc_name):
        return self.mock_if.mock_param_disappear(svc_name)

PyrosBase.register(PyrosMock)
