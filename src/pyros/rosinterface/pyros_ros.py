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
    def __init__(self, name=None, argv=None, base_path=None):
        super(PyrosROS, self).__init__(name=name or 'pyros_ros')
        # removing name from argv to avoid overriding specified name unintentionally
        argv = [arg for arg in (argv or []) if not arg.startswith('__name:=')]
        # protecting rospy from unicode
        self.str_argv = [unicodedata.normalize('NFKD', arg).encode('ascii', 'ignore') if isinstance(arg, unicode) else str(arg) for arg in argv]
        self.base_path = base_path  # used for setup in actual separate process dynamically

        self.enable_cache = False
        self.ros_if = None
        self.ros_if_params = None   # for delayed reinit()

    # TODO: get rid of this to need one less client-node call
    # we need make the message type visible to client,
    # and have him convert to a python structure that we can then convert to a message
    # dynamically right when calling the service.
    def msg_build(self, connec_name):
        msg = None
        if self.ros_if:
            if connec_name in self.ros_if.topics.keys():
                input_msg_type = self.ros_if.topics.get(connec_name, None).rostype
                msg = input_msg_type()
            elif connec_name in self.ros_if.services.keys():
                input_msg_type = self.ros_if.services.get(connec_name, None).rostype_req
                msg = input_msg_type()
        return msg

    # These should match the design of RostfulClient and Protocol so we are consistent between pipe and python API
    def topic(self, name, msg_content=None):
        try:
            msg = self.msg_build(name)
            if self.ros_if and name in self.ros_if.topics.keys():
                if msg_content is not None:
                    msgconv.populate_instance(msg_content, msg)
                    self.ros_if.topics.get(name, None).publish(msg)
                    msg = None  # consuming the message
                else:
                    res = self.ros_if.topics.get(name, None).get(consume=False)
                    msg = msgconv.extract_values(res) if res else res
            return msg
        except msgconv.FieldTypeMismatchException, e:
            rospy.logerr("[{name}] : field type mismatch {e}".format(name=__name__, e=e))
            raise

    def topics(self):
        topics_dict = {}
        if self.ros_if:
            for t, tinst in self.ros_if.topics.iteritems():
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
            if self.ros_if and name in self.ros_if.services.keys():
                resp = self.ros_if.services.get(name, None).call(rqst)
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
        if self.ros_if:
            for s, sinst in self.ros_if.services.iteritems():
                services_dict[s] = sinst.asdict()
        return services_dict

    def param(self, name, value=None):
        if self.ros_if and name in self.ros_if.params.keys():
            if value is not None:
                self.ros_if.params.get(name, None).set(value)
                value = None  # consuming the message
            else:
                value = self.ros_if.params.get(name, None).get()
        return value

    def params(self):
        params_dict = {}
        if self.ros_if:
            for p, pinst in self.ros_if.params.iteritems():
                params_dict[p] = pinst.asdict()
        return params_dict

    def reinit(self, services=None, topics=None, params=None, enable_cache=None):
        # this needs to be available just after __init__, however we need the ros_if to be present
        self.ros_if_params = (services, topics, params, enable_cache)
        if self.ros_if:
            self.ros_if.reinit(*self.ros_if_params)

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

        self.enable_cache = rospy.get_param('~enable_cache', False)
        self.ros_if = RosInterface(enable_cache=self.enable_cache)

        if self.ros_if_params:
            self.ros_if.reinit(*self.ros_if_params)

        # we initialize the node here, in subprocess, passing ros parameters.
        # disabling signal to avoid overriding callers behavior
        rospy.init_node(self.name, argv=self.str_argv, disable_signals=True)
        rospy.loginfo('PyrosROS {name} node started with args : {argv}'.format(name=self.name, argv=self.str_argv))

        # TODO : install shutdown hook to shutdown if detected

        try:
            logging.debug("zmp[{name}] running, pid[{pid}]".format(name=__name__, pid=os.getpid()))

            super(PyrosROS, self).run()

            logging.debug("zmp[{name}] shutdown, pid[{pid}]".format(name=__name__, pid=os.getpid()))

        except KeyboardInterrupt:
            rospy.logwarn('PyrosROS node stopped by keyboad interrupt')

    def shutdown(self, join=True):
        """
        Clean shutdown of the node.
        :param join: optionally wait for the process to end (default : True)
        :return: None
        """
        super(PyrosROS, self).shutdown(join)

    def update_throttled(self):
        """
        Update function to call from a looping thread.
        """
        self.ros_if.update()


PyrosBase.register(PyrosROS)
