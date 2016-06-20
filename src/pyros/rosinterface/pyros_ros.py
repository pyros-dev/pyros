from __future__ import absolute_import

import collections
import time

# ROS Environment should already be setup before importing this
# But beware master might not be started yet
import pyros_setup
import rospy
import sys

from .ros_interface import RosInterface

from pyros.baseinterface import PyrosBase
import ast
import os
import logging
import unicodedata

# TODO : move cfg, srv, and other ROS specific stuff in the same rosinterface module ?
# TODO : get rid of cfg, srv and other things that require ROS build system, so that we can have pyros as pure python package ?

from . import message_conversion as msgconv
from .topic import TopicBack


class PyrosROS(PyrosBase):
    """
    Interface with ROS.
    """
    def __init__(self, name=None, argv=None, base_path=None, args=None, kwargs=None):
        super(PyrosROS, self).__init__(name=name or 'pyros_ros', interface_class=RosInterface, args=args or (), kwargs=kwargs or {})
        # removing name from argv to avoid overriding specified name unintentionally
        argv = [arg for arg in (argv or []) if not arg.startswith('__name:=')]
        # protecting rospy from unicode
        self.str_argv = [unicodedata.normalize('NFKD', arg).encode('ascii', 'ignore') if isinstance(arg, unicode) else str(arg) for arg in argv]
        self.base_path = base_path  # used for setup in actual separate process dynamically

    # TODO: get rid of this to need one less client-node call
    # we need make the message type visible to client,
    # and have him convert to a python structure that we can then convert to a message
    # dynamically right when calling the service.
    def msg_build(self, connec_name):
        msg = None
        if self.interface:
            if connec_name in self.interface.topics.keys():
                input_msg_type = self.interface.topics.get(connec_name, None).rostype
                msg = input_msg_type()
            elif connec_name in self.interface.services.keys():
                input_msg_type = self.interface.services.get(connec_name, None).rostype_req
                msg = input_msg_type()
        return msg

    # These should match the design of RostfulClient and Protocol so we are consistent between pipe and python API
    def topic(self, name, msg_content=None):
        try:
            msg = self.msg_build(name)
            if self.interface and name in self.interface.topics.keys():
                if msg_content is not None:
                    msgconv.populate_instance(msg_content, msg)
                    self.interface.topics.get(name, None).publish(msg)
                    msg = None  # consuming the message
                else:
                    res = self.interface.topics.get(name, None).get(consume=False)
                    msg = msgconv.extract_values(res) if res else res
            return msg
        except msgconv.FieldTypeMismatchException, e:
            rospy.logerr("[{name}] : field type mismatch {e}".format(name=__name__, e=e))
            raise

    def topics(self):
        topics_dict = {}
        if self.interface:
            for t, tinst in self.interface.topics.iteritems():
                topics_dict[t] = tinst.asdict()
        return topics_dict

    def service(self, name, rqst_content=None):
        try:
            resp_content = None
            rqst = self.msg_build(name)
            msgconv.populate_instance(rqst_content, rqst)

            # FIXME : if the service is not exposed this returns None.
            # Cost a lot time to find the reason since client code doesnt check the answer.
            # Maybe returning error is better ?
            if self.interface and name in self.interface.services.keys():
                resp = self.interface.services.get(name, None).call(rqst)
                resp_content = msgconv.extract_values(resp)
            return resp_content

        except rospy.ServiceException, e:
            rospy.logerr("[{name}] : service exception {e}".format(name=__name__, e=e))
            raise
        except msgconv.FieldTypeMismatchException, e:
            rospy.logerr("[{name}] : field type mismatch {e}".format(name=__name__, e=e))
            raise
        except msgconv.NonexistentFieldException, e:
            rospy.logerr("[{name}] : non existent field {e}".format(name=__name__, e=e))
            raise

    ###

    def services(self):
        services_dict = {}
        if self.interface:
            for s, sinst in self.interface.services.iteritems():
                services_dict[s] = sinst.asdict()
        return services_dict

    def param(self, name, value=None):
        if self.interface and name in self.interface.params.keys():
            if value is not None:
                self.interface.params.get(name, None).set(value)
                value = None  # consuming the message
            else:
                value = self.interface.params.get(name, None).get()
        return value

    def params(self):
        params_dict = {}
        if self.interface:
            for p, pinst in self.interface.params.iteritems():
                params_dict[p] = pinst.asdict()
        return params_dict

    def setup(self, *args, **kwargs):
        """
        Dynamically reset the interface to expose the services / topics / params whose names are passed as args
        :param services:
        :param topics:
        :param params:
        :param enable_cache
        :return:
        """

        # we add params from ros args to kwargs if it is not there yet
        kwargs.setdefault('services', list(set(ast.literal_eval(rospy.get_param('~services', "[]")))))
        kwargs.setdefault('topics', list(set(ast.literal_eval(rospy.get_param('~topics', "[]")))))
        kwargs.setdefault('params', list(set(ast.literal_eval(rospy.get_param('~params', "[]")))))
        kwargs.setdefault('enable_cache', rospy.get_param('~enable_cache', False))

        super(PyrosROS, self).setup(*args, **kwargs)

    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """
        # Environment should be setup here if needed ( we re in another process ).
        sys.modules["pyros_setup"] = pyros_setup.delayed_import_auto(distro='indigo', base_path=self.base_path)

        # master has to be running here or we just wait for ever
        m, _ = pyros_setup.get_master(spawn=False)
        while not m.is_online():
            time.sleep(0.5)

        # we initialize the node here, in subprocess, passing ros parameters.
        # disabling signal to avoid overriding callers behavior
        rospy.init_node(self.name, argv=self.str_argv, disable_signals=True)
        rospy.loginfo('PyrosROS {name} node started with args : {argv}'.format(name=self.name, argv=self.str_argv))

        # TODO : install shutdown hook to shutdown if detected

        try:
            # this spins with regular frequency
            super(PyrosROS, self).run()  # we override parent run to add one argument to ros interface

        except KeyboardInterrupt:
            rospy.logwarn('PyrosROS node stopped by keyboard interrupt')

    def shutdown(self, join=True):
        """
        Clean shutdown of the node.
        :param join: optionally wait for the process to end (default : True)
        :return: None
        """
        super(PyrosROS, self).shutdown(join)


PyrosBase.register(PyrosROS)
