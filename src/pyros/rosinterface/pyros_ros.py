from __future__ import absolute_import

import rospy

from .ros_interface import RosInterface
try:
    from .roconinterface.rocon_interface import RoconInterface
    _ROCON_AVAILABLE = True
except ImportError, e:
    import logging
    logging.warn("Error: could not import RoconInterface - disabling. %s" % e)
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

# TODO : move cfg, srv, and other ROS specific stuff in the same rosinterface module ?

import pyros.srv as srv
from . import message_conversion as msgconv


class PyrosROS(PyrosBase):
    """
    Interface with ROS.
    """
    def __init__(self, name, argv):
        super(PyrosROS, self).__init__(name='pyros-ROS')
        self.argv = argv
        enable_rocon = rospy.get_param('~enable_rocon', False)
        self.enable_rocon = enable_rocon

        self.ros_if = RosInterface()

        if _ROCON_AVAILABLE and self.enable_rocon:

            rospy.logerr("ENABLE_ROCON IS TRUE IN INIT!!")
            self.rocon_if = RoconInterface(self.ros_if)
            pass
        else:
            self.rocon_if = None


        ##############################################################################################
        #### Helpers in case we need to listen to someone talking from a different process
        # Needed because of limitation in rospy that we cannot publish on topic from different process
        # Note : It should be working fine for services however
        # TODO : change this to use pure python code to avoid confusion ( for ex. using multiprocessing lib )
        ##############################################################################################

        def start_rapp(req):  # Keep this minimal
            rospy.logwarn("""Requesting Rapp Start {rapp}: """.format(
                rapp=req.rapp_name
            ))
            #normalizing names... ( somewhere else ?)
            #service_name = unicodedata.normalize('NFKD', req.service_name).encode('ascii', 'ignore')
            #service is raw str
            if req.rapp_name[0] == '/':
                req.rapp_name = req.rapp_name[1:]

            if self.rocon_if:
                #TMP
                if req.rapp_name.split('/')[0] in self.rocon_if.rapps_namespaces:
                    self.rocon_if.start_rapp(req.rapp_name.split('/')[0], "/".join(req.rapp_name.split('/')[1:]))

            res = True

            return srv.StartRappResponse(res)

        def stop_rapp(req):  # Keep this minimal
            rospy.logwarn("""Requesting Rapp Stop: """)

            if self.rocon_if:
                #TMP
                self.rocon_if.stop_rapp()

            res = {"stopped": True}
            output_data = json.dumps(res)
            return srv.StopRappResponse(output_data)

        self.RappStartService = rospy.Service('~start_rapp', srv.StartRapp, start_rapp)
        self.RappStopService = rospy.Service('~stop_rapp', srv.StopRapp, stop_rapp)

        self.provides(self.msg_build)
        self.provides(self.topic)
        self.provides(self.topics)
        self.provides(self.service)
        self.provides(self.services)
        self.provides(self.param)
        self.provides(self.params)
        #self.provides(self.interactions)
        #self.provides(self.namespaces)
        #self.provides(self.interaction)
        #self.provides(self.has_rocon)

        ####

    def msg_build(self, connec_name):
        msg = None
        if self.ros_if:
            if self.ros_if.get_topic(connec_name):
                input_msg_type = self.ros_if.get_topic(connec_name).rostype
                msg = input_msg_type()
            elif self.ros_if.get_service(connec_name):
                input_msg_type = self.ros_if.get_service(connec_name).rostype_req
                msg = input_msg_type()
        return msg

    # These should match the design of RostfulClient and Protocol so we are consistent between pipe and python API
    def topic(self, name, msg_content=None):
        try:
            msg = self.msg_build(name)
            if self.ros_if and self.ros_if.get_topic(name):
                if msg_content is not None:
                    msgconv.populate_instance(msg_content, msg)
                    self.ros_if.get_topic(name).publish(msg)
                    msg = None  # consuming the message
                else:
                    res = self.ros_if.get_topic(name).get(consume=False)
                    msg = msgconv.extract_values(res) if res else res
            return msg
        except msgconv.FieldTypeMismatchException, e:
            rospy.logerr("Rostful Node : field type mismatch %r" % e)

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

            if self.ros_if and self.ros_if.get_service(name):
                resp = self.ros_if.get_service(name).call(rqst)
                resp_content = msgconv.extract_values(resp)
            return resp_content

        except rospy.ServiceException, e:
            rospy.logerr("Rostful Node : service exception %r" % e)
        except msgconv.FieldTypeMismatchException, e:
            rospy.logerr("Rostful Node : field type mismatch %r" % e)
        except msgconv.NonexistentFieldException, e:
            rospy.logerr("Rostful Node : non existent field %r" % e)

    ###

    def services(self):
        services_dict = {}
        if self.ros_if:
            for s, sinst in self.ros_if.services.iteritems():
                services_dict[s] = sinst.asdict()
        return services_dict

    def param(self, name, value=None):
        if self.ros_if and self.ros_if.get_param(name):
            if value is not None:
                self.ros_if.get_param(name).set(value)
                value = None  # consuming the message
            else:
                value = self.ros_if.get_param(name).get()
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

    def run(self):
        """
        Running in a zmp.Node process, providing zmp.services
        """
        # we initialize the node here, in subprocess, passing ros parameters.
        # disabling signal to avoid overriding callers behavior
        rospy.init_node(self.name, argv=self.argv, disable_signals=True)
        rospy.logwarn('rostful node started with args : %r', self.argv)

        # Create a dynamic reconfigure server ( needs to be done after node_init )
        self.server = Server(PyrosConfig, self.reconfigure)

        #TODO : install shutdown hook to shutdown if detected

        logging.debug("zmp[{name}] running, pid[{pid}]".format(name=self.name, pid=os.getpid()))

        super(PyrosROS, self).run()

        logging.debug("zmp[{name}] shutdown, pid[{pid}]".format(name=self.name, pid=os.getpid()))


    def update(self):
        """
        Update function to call from a looping thread.
        """

        self.ros_if.update()

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        rospy.logwarn("""Reconfigure Request: \renable_rocon : {enable_rocon}""".format(**config))
        self.enable_rocon = config["enable_rocon"]

        if not self.rocon_if and self.enable_rocon:
            rospy.logerr("ENABLE_ROCON IS TRUE IN RECONF !!")
            self.rocon_if = RoconInterface(self.ros_if)

        config = self.ros_if.reconfigure(config, level)

        if self.rocon_if:
            config = self.rocon_if.reconfigure(config, level)

        return config



PyrosBase.register(PyrosROS)
