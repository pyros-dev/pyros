from __future__ import absolute_import

import ast
import json
import os
import logging
import time

import pyzmp
# python protocol should be usable without ROS.
from pyros.baseinterface import PyrosBase
from .mockinterface import MockInterface


class PyrosMock(PyrosBase):
    """
    Mock Interface in pure python ( No ROS needed ).

    """

    # TODO : we probably want to reuse the standard mock module here...
    def __init__(self, name=None, args=None, kwargs=None):
        name = name or 'pyros-mock'
        super(PyrosMock, self).__init__(name, interface_class=MockInterface, args=args or (), kwargs=kwargs or {})
        self._topic_msg = {}  # storage for the echo topic
        self._param_val = {}  # storage for the test param
        self._stop_event = None  # stop_event to signal the thread for soft shutdown
        self._spinner = None  # thread instance

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
        """
        :return: the list of topics we interfaced with ( not the list of all available topics )
        """
        return self.interface.topics

    # a simple string echo service
    def service(self, name, rqst_content=None):
        # simulating a strict message typing backend
        #if not isinstance(rqst_content, str):
        #    raise Exception("Request Content Not Expected Type !")
        resp_content = rqst_content
        return resp_content

    def services(self):
        """
        :return: the list of services we interfaced with ( not the list of all available services )
        """
        return self.interface.services

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
        """
        :return: the list of params we interfaced with ( not the list of all available params )
        """
        return self.interface.params

    def setup(self, services=None, topics=None, params=None):
        super(PyrosMock, self).setup(services, topics, params)

    # #
    # # Specific Mock hack
    # # The interface should not be replaced on run ( to keep mock data )
    # def run(self, services=None, topics=None, params=None):
    #     """
    #     Running in a pyzmp.Node process, providing pyzmp.services
    #     """
    #
    #     logging.debug("pyros node running, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))
    #
    #     # Initialization ( here in child )
    #     # None passed in argument means empty list ( different than reinit meaning )
    #
    #     super(PyrosBase, self).run()
    #
    #     logging.debug("pyros node shutdown, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

PyrosBase.register(PyrosMock)
