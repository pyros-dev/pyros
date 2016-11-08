from __future__ import absolute_import

import os

import six
import time


import sys

from .basenode import PyrosBase
import logging
import unicodedata

from .utils import deprecated


from . import config


class PyrosROS(PyrosBase):

    """
    Interface with ROS.
    """
    #: Default configuration parameters.
    _default_config = {
        'TOPICS': [],
        'SERVICES': [],
        'PARAMS': [],
        'ROS_USE_CONNECTION_CACHE': False,
        'ROS_CONNECTION_CACHE_LIST_TOPIC': "/rocon/connection_cache/list",
        'ROS_CONNECTION_CACHE_DIFF_TOPIC': "/rocon/connection_cache/diff",
    }

    def __init__(self, name=None, argv=None, pyros_config=None, args=None, kwargs=None):
        """
        Initializes a ROS node inside a zmp.Node, providing all the easy multiprocess communication of ZMP for it
        :param name: the name of the process
        :param argv: the arguments passed for its creation (ROS args)
        :param pyros_config: the configuration to use to configure pyros
        :param args: arguments to pass to the interface_class initializer. If provided, the first one is used as the ROS node name.
        :param kwargs: keyword arguments to pass to the interface_class initializer. If provided the 'argv' key will override the other 'argv' parameter.
        """

        # removing name from argv to avoid overriding specified name unintentionally
        argv = [arg for arg in (argv or []) if not arg.startswith('__name:=')]
        # protecting rospy from unicode
        str_argv = [
            unicodedata.normalize('NFKD', arg).encode('ascii', 'ignore') if isinstance(arg, unicode) else str(arg) for
            arg in argv]

        # argv is special :
        # They can be set only once, since they are passed to rospy.node_init in the interface.
        # That behavior is implemented in setup()
        self.argv = str_argv

        args = args or ()
        kwargs = kwargs or {}

        super(PyrosROS, self).__init__(
            name=name or 'pyros_ros',  # Careful with name : we cannot restart a node with the same name again...
            interface_class=(__package__, '.rosinterface', 'RosInterface'),  # lazy class evaluation to delay imports
            args=args,  # we want to pass the name to the interface to init the node with that name
            kwargs=kwargs,
            instance_relative_config=True,
            default_config=self._default_config)  # we pass our specific default config

        # overriding default config with parameter provided
        self.config_handler.configure(config)  # configuring with our package default
        if pyros_config:
            self.config_handler.configure(pyros_config)  # configuring with argument passed form user





    # TODO: get rid of this to need one less client-node call
    # we need make the message type visible to client,
    # and have him convert to a python structure that we can then convert to a message
    # dynamically right when calling the service.
    @deprecated
    def msg_build(self, connec_name):
        msg = None
        if self.interface:
            if connec_name in self.interface.topics.keys():
                input_msg_type = self.interface.topics.get(connec_name).rostype
                msg = input_msg_type()
            elif connec_name in self.interface.services.keys():
                input_msg_type = self.interface.services.get(connec_name).rostype_req
                msg = input_msg_type()
        return msg

    # These should match the design of RostfulClient and Protocol so we are consistent between pipe and python API
    #BWCOMPAT
    def topic(self, name, msg_content=None):
        res = None
        if self.interface:
            if msg_content is not None and name in self.interface.subscribers.keys():
                self.interface.subscribers.get(name).publish(msg_content)
            elif name in self.interface.publishers.keys():
                res = self.interface.publishers.get(name).get(consume=False)
        return res

    def topics(self):
        topics_dict = {}
        if self.interface:
            # merging pubs and subs for BWCOMPAT (like in pyros_mock)
            topics = self.interface.publishers.copy()
            topics.update(self.interface.subscribers)

            for t, tinst in six.iteritems(topics):
                topics_dict[t] = tinst.asdict()
        return topics_dict

    def publisher(self, name):
        res = None
        if self.interface and name in self.interface.publishers.keys():
            res = self.interface.publishers.get(name).get(consume=False)
        return res

    def publishers(self):
        publishers_dict = {}
        if self.interface:
            for t, tinst in six.iteritems(self.interface.publishers):
                publishers_dict[t] = tinst.asdict()
        return publishers_dict

    def subscriber(self, name, msg_content):
        res = None
        if self.interface and name in self.interface.subscribers.keys():
            self.interface.subscribers.get(name).publish(msg_content)
        return res

    def subscribers(self):
        subscribers_dict = {}
        if self.interface:
            for t, tinst in six.iteritems(self.interface.subscribers):
                subscribers_dict[t] = tinst.asdict()
        return subscribers_dict

    def service(self, name, rqst_content=None):
        resp_content = None

        # FIXME : if the service is not exposed this returns None.
        # Cost a lot time to find the reason since client code doesnt check the answer.
        # Maybe returning error is better ?
        if self.interface and name in self.interface.services.keys():
            resp_content = self.interface.services.get(name).call(rqst_content)
        return resp_content

    ###

    def services(self):
        services_dict = {}
        if self.interface:
            for s, sinst in six.iteritems(self.interface.services):
                services_dict[s] = sinst.asdict()
        return services_dict

    def param(self, name, value=None):
        if self.interface and name in self.interface.params.keys():
            if value is not None:
                self.interface.params.get(name).set(value)
                value = None  # consuming the message
            else:
                value = self.interface.params.get(name).get()
        return value

    def params(self):
        params_dict = {}
        if self.interface:
            for p, pinst in six.iteritems(self.interface.params):
                params_dict[p] = pinst.asdict()
        return params_dict

    def setup(self, publishers=None, subscribers=None, services=None, topics=None, params=None, enable_cache=False):
        """
        Service to dynamically setup the node.
        Node we cannot pass the name here as it should be set only once, the first time
        """
        # we get self.name and self.argv from the duplicated parent process memory.
        # this will create self.interface
        super(PyrosROS, self).setup(node_name=self.name, publishers=publishers, subscribers=subscribers, services=services, topics=topics, params=params, enable_cache=enable_cache, argv=self.argv)

    def run(self, *args, **kwargs):
        """
        Running in a zmp.Node process, providing zmp.services
        """

        # TODO : install shutdown hook to shutdown if detected

        try:
            # This will instantiate the rosinterface, which will initialize the ros node.
            # this is all done here in the child process, to not pollute the original process context.
            # this spins with regular frequency
            super(PyrosROS, self).run(*args, **kwargs)  # we override parent run to add one argument to ros interface

        except KeyboardInterrupt:
            logging.warn('PyrosROS node stopped by keyboard interrupt')

    def shutdown(self, join=True, timeout=None):
        """
        Clean shutdown of the node.
        :param join: optionally wait for the process to end (default : True)
        :return: None
        """
        super(PyrosROS, self).shutdown(join, timeout=timeout)


PyrosBase.register(PyrosROS)
