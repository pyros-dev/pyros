from __future__ import absolute_import

import ast
import json
import os
import logging
import time
import abc

import zmp
# python protocol should be usable without ROS.
from ..pyros_prtcl import MsgBuild, Topic, Service, Param, ServiceList, ServiceInfo, TopicList, TopicInfo, ParamList, ParamInfo, Interaction, Interactions, InteractionInfo, Namespaces, NamespaceInfo, Rocon
from .baseinterface import BaseInterface


# TODO : service interface for mock to be able to dynamically trigger services / topics / params appearing & disappearing
class PyrosBase(zmp.Node):
    """
    Mock Interface in pure python ( No ROS needed ).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name=None, context_manager=None):
        """
        :param name: name of the node
        :param context_manager: a context manager to enter when starting, and exit when the node stops
        :return:
        """
        super(PyrosBase, self).__init__(name or 'pyros', context_manager=context_manager)

        self.last_update = 0
        self.update_interval = 1  # seconds to wait between each update

        self.provides(self.msg_build)
        self.provides(self.topic)
        self.provides(self.topics)
        self.provides(self.service)
        self.provides(self.services)
        self.provides(self.param)
        self.provides(self.params)
        self.provides(self.reinit)
        pass

    # These should match the design of PyrosClient and Protocol so we are consistent between pipe and python API
    @abc.abstractmethod
    def msg_build(self, name):
        return

    @abc.abstractmethod
    def topic(self, name, msg_content=None):
        return

    @abc.abstractmethod
    def topics(self):
        return

    # a simple string echo service
    @abc.abstractmethod
    def service(self, name, rqst_content=None):
        return

    @abc.abstractmethod
    def services(self):
        return

    @abc.abstractmethod
    def param(self, name, value=None):
        return

    @abc.abstractmethod
    def params(self):
        return

    @abc.abstractmethod
    def reinit(self):
        return

    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """

        logging.debug("pyros node running, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

        super(PyrosBase, self).run()

        logging.debug("pyros node shutdown, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

    def shutdown(self, join=True):
        """
        Clean shutdown of the node.
        :param join: optionally wait for the process to end (default : True)
        :return: None
        """

        super(PyrosBase, self).shutdown(join)

    def update(self, timedelta):
        """
        Update function to call from a looping thread.
        :param timedelta: the time past since the last update call
        """
        self.last_update += timedelta
        if self.last_update > self.update_interval:
            self.last_update = 0
            self.update_throttled()

    def update_throttled(self):
        """
        An update method that is only run every self.update_interval
        :return:
        """
        pass

