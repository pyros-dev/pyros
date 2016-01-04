from __future__ import absolute_import

import collections
import rospy
import time

from .ros_interface import RosInterface

try:
    import rocon_std_msgs.msg as rocon_std_msgs
except ImportError:
    print("Cannot import rocon_std_msgs needed to use ConnectionCache.")
    rocon_std_msgs = None

try:
    from .roconinterface.rocon_interface import RoconInterface
    _ROCON_AVAILABLE = True
except ImportError, e:
    import logging
    logging.warn("Error: could not import RoconInterface - disabling. {0!s}".format(e))
    _ROCON_AVAILABLE = False

import zmp
from ..pyros_prtcl import MsgBuild, Topic, Service, Param, TopicInfo, ServiceInfo, ParamInfo, Rocon, InteractionInfo, NamespaceInfo

from pyros.baseinterface import PyrosBase
from dynamic_reconfigure.server import Server
from pyros.cfg import PyrosConfig
import ast
import json
import os
import logging
import unicodedata

# TODO : move cfg, srv, and other ROS specific stuff in the same rosinterface module ?

import pyros.srv as srv
from . import message_conversion as msgconv, TopicBack


class PyrosROS(PyrosBase):
    """
    Interface with ROS.
    """
    def __init__(self, name=None, argv=None, dynamic_reconfigure=True):
        super(PyrosROS, self).__init__(name=name or 'pyros_ros')
        # removing name from argv to avoid overriding specified name unintentionally
        argv = [arg for arg in (argv or []) if not arg.startswith('__name:=')]
        # protecting rospy from unicode
        self.str_argv = [unicodedata.normalize('NFKD', arg).encode('ascii', 'ignore') if isinstance(arg, unicode) else str(arg) for arg in argv]
        self.dynamic_reconfigure = dynamic_reconfigure

        enable_rocon = rospy.get_param('~enable_rocon', False)
        self.enable_rocon = enable_rocon

        self.ros_if = RosInterface()

        if _ROCON_AVAILABLE and self.enable_rocon:

            rospy.logerr("ENABLE_ROCON IS TRUE IN INIT!!")
            self.rocon_if = RoconInterface(self.ros_if)
            pass
        else:
            self.rocon_if = None

        # def start_rapp(req):  # Keep this minimal
        #     rospy.logwarn("""Requesting Rapp Start {rapp}: """.format(
        #         rapp=req.rapp_name
        #     ))
        #     #normalizing names... ( somewhere else ?)
        #     #service_name = unicodedata.normalize('NFKD', req.service_name).encode('ascii', 'ignore')
        #     #service is raw str
        #     if req.rapp_name[0] == '/':
        #         req.rapp_name = req.rapp_name[1:]
        #
        #     if self.rocon_if:
        #         #TMP
        #         if req.rapp_name.split('/')[0] in self.rocon_if.rapps_namespaces:
        #             self.rocon_if.start_rapp(req.rapp_name.split('/')[0], "/".join(req.rapp_name.split('/')[1:]))
        #
        #     res = True
        #
        #     return srv.StartRappResponse(res)
        #
        # def stop_rapp(req):  # Keep this minimal
        #     rospy.logwarn("""Requesting Rapp Stop: """)
        #
        #     if self.rocon_if:
        #         #TMP
        #         self.rocon_if.stop_rapp()
        #
        #     res = {"stopped": True}
        #     output_data = json.dumps(res)
        #     return srv.StopRappResponse(output_data)

        #self.RappStartService = rospy.Service('~start_rapp', srv.StartRapp, start_rapp)
        #self.RappStopService = rospy.Service('~stop_rapp', srv.StopRapp, stop_rapp)

        #self.provides(self.interactions)
        #self.provides(self.namespaces)
        #self.provides(self.interaction)
        #self.provides(self.has_rocon)

        ####

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

    def interaction(self, name):
        if self.rocon_if and name in self.rocon_if.interactions:
            self.rocon_if.request_interaction(name)

        return None
            
    def interactions(self):
        if self.rocon_if:
            ir = self.rocon_if.interactions
            inter_dict = {}
            for intr in ir:
                inter_dict[intr] = InteractionInfo(name=intr.name, display_name=intr.display_name)

            return inter_dict
        return {}

    def namespaces(self):
        if self.rocon_if:
            ns = self.rocon_if.rapps_namespaces
            rapp_dict = {}
            for rapp in ns:
                rapp_dict[rapp] = NamespaceInfo(name=rapp.name)

            return rapp_dict
                
        return {}

    def has_rocon(self):
        return True if self.rocon_if else False

    def reinit(self, services, topics, params):
        return self.ros_if.reinit(services, topics, params)

    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """
        # we initialize the node here, in subprocess, passing ros parameters.
        # disabling signal to avoid overriding callers behavior
        rospy.init_node(self.name, argv=self.str_argv, disable_signals=True)
        rospy.logwarn('PyrosROS {name} node started with args : {argv}'.format(name=self.name, argv=self.str_argv))

        if self.dynamic_reconfigure:
            # Create a dynamic reconfigure server ( needs to be done after node_init )
            self.server = Server(PyrosConfig, self.reconfigure)

        # TODO : install shutdown hook to shutdown if detected

        try:
            logging.debug("zmp[{name}] running, pid[{pid}]".format(name=__name__, pid=os.getpid()))

            super(PyrosROS, self).run()

            logging.debug("zmp[{name}] shutdown, pid[{pid}]".format(name=__name__, pid=os.getpid()))

        except KeyboardInterrupt:
            rospy.logwarn('PyrosROS node stopped by keyboad interrupt')

    def update_throttled(self):
        """
        Update function to call from a looping thread.
        """
        self.ros_if.update()

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):

        new_services = None
        new_topics = None
        new_params = None
        try:
            # convert new services to a set and then back to a list to ensure uniqueness
            new_services = list(set(ast.literal_eval(config["services"])))
        except ValueError:
            rospy.logwarn('[{name}] Ignored list {services} containing malformed service strings. Fix your input!'.format(name=__name__, **config))
        try:
            # convert new topics to a set and then back to a list to ensure uniqueness
            new_topics = list(set(ast.literal_eval(config["topics"])))
        except ValueError:
            rospy.logwarn('[{name}] Ignored list {topics} containing malformed topic strings. Fix your input!'.format(name=__name__, **config))
        try:
            # convert new params to a set and then back to a list to ensure uniqueness
            new_params = list(set(ast.literal_eval(config["params"])))
        except ValueError:
            rospy.logwarn('[{name}] Ignored list {params} containing malformed param strings. Fix your input!'.format(name=__name__, **config))

        self.enable_rocon = config.get('enable_rocon', False)

        rospy.logwarn("""[{name}] Interface Reconfigure Request:
    services : {services}
    topics : {topics}
    params : {params}
    enable_rocon : {enable_rocon}
        """.format(name=__name__,
                   topics="\n" + "- ".rjust(10) + "\n\t- ".join(new_topics) if new_topics else "None",
                   services="\n" + "- ".rjust(10) + "\n\t- ".join(new_services) if new_services else "None",
                   params="\n" + "- ".rjust(10) + "\n\t- ".join(new_params) if new_params else "None",
                   enable_rocon=config.get('enable_rocon', False),
                   ))

        if not self.rocon_if and self.enable_rocon:
            rospy.logerr("ENABLE_ROCON IS TRUE IN RECONF !!")
            self.rocon_if = RoconInterface(self.ros_if)

        self.reinit(new_services, new_topics, new_params)

        if self.rocon_if:
            config = self.rocon_if.reconfigure(config, level)

        return config



PyrosBase.register(PyrosROS)
