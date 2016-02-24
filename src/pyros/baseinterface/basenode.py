from __future__ import absolute_import

import ast
import json
import os
import logging
import time
import abc

import zmp

from . import BaseInterface


class PyrosBase(zmp.Node):
    """
    Mock Interface in pure python ( No ROS needed ).
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name=None, interface_class=BaseInterface, context_manager=None, args=None, kwargs=None):
        """
        :param name: name of the node
        :param context_manager: a context manager to enter when starting, and exit when the node stops
        :return:
        """
        super(PyrosBase, self).__init__(name or 'pyros', context_manager=context_manager, args=args or (), kwargs=kwargs or {})

        self.last_update = 0
        self.update_interval = 1  # seconds to wait between each update

        self.provides(self.msg_build)
        self.provides(self.topic)
        self.provides(self.topics)
        self.provides(self.service)
        self.provides(self.services)
        self.provides(self.param)
        self.provides(self.params)
        self.provides(self.setup)

        # TODO : maybe assert callable, or class here ? Goal is to avoid late error on run()...
        self.interface_class = interface_class
        self.interface = None  # delayed interface creation
        # interface should be created in child process only to maintain isolation.

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
    def setup(self, *args, **kwargs):
        """
        Dynamically reset the interface to expose the services / topics / params whose names are passed as args
        :param services:
        :param topics:
        :param params:
        :return:
        """

        # priority for arguments in setup() call :
        # 1. setup ( args ) if called directly by service -> override any other args
        # 2. process ( _args ) if passed from parent process -> override environment provided args
        # 3. others ( like ros args or config args coming from file etc. )
        # TODO : replace rosargs by config file ??? -> goal is simplicity from user perspective.

        self.interface = self.interface_class(*args, **kwargs)

    # TODO : change this running interface to have a delegate zmp node ( instead of motherclass )
    # Maybe this will help when passing custom argument from child classes to child processes...
    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """

        logging.debug("pyros node running, zmp[{name}] pid[{pid}]".format(name=self.name, pid=os.getpid()))

        # Initialization ( here in child )
        # None passed in argument means empty list ( different than reinit meaning )

        # getting preset params from parent process memory
        self.setup(*self._args, **self._kwargs)

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
        It updates the interface state
        :return:
        """
        self.interface.update()

