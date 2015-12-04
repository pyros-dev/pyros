from __future__ import absolute_import

import ast
import json
import os
import logging
import time

import zmp  # TODO : This shouldnt be needed here. get rid of it !
# python protocol should be usable without ROS.
from ..pyros_prtcl import MsgBuild, Topic, Service, Param, ServiceList, ServiceInfo, TopicList, TopicInfo, ParamList, ParamInfo, Interaction, Interactions, InteractionInfo, Namespaces, NamespaceInfo, Rocon
from .mockinterface import MockInterface


class PyrosMock(zmp.Node):
    """
    Mock Interface in pure python ( No ROS needed ).
    """
    def __init__(self, name='pyros-mock'):
        super(PyrosMock, self).__init__(name)
        self._topic_msg = {}  # storage for the echo topic
        self._param_val = {}  # storage for the test param
        self._stop_event = None  # stop_event to signal the thread for soft shutdown
        self._spinner = None  # thread instance

        self.mock_if = MockInterface()

        self.provides(self.msg_build)
        self.provides(self.topic)
        self.provides(self.topic_list)
        self.provides(self.service)
        self.provides(self.service_list)
        self.provides(self.param)
        self.provides(self.param_list)
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


