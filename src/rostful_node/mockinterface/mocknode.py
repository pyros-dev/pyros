from __future__ import absolute_import, print_function

from .mockinterface import MockInterface

from .rostful_prtcl import MsgBuild, Topic, Service, Param, TopicInfo, ServiceInfo, ParamInfo, Rocon, InteractionInfo, NamespaceInfo
from .rostful_mock import RostfulMock

from dynamic_reconfigure.server import Server
from rostful_node.cfg import RostfulNodeConfig

import rostful_node.srv as srv
from rosinterface import message_conversion as msgconv
from rosinterface.action import ActionBack

from multiprocessing import Pipe, Process, Event
"""
MockNode
"""

class MockNode(RostfulMock):
    def __init__(self):
        super(RostfulNode, self).__init__()
        enable_rocon = rospy.get_param('~enable_rocon', False)
        self.enable_rocon = enable_rocon

        self.ros_if = RosInterface()

        # Create a dynamic reconfigure server.
        self.server = Server(RostfulNodeConfig, self.reconfigure)

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

    def topic_list(self):
        if self.ros_if:
            # get the dict of topics, and then extract only the relevant
            # information from the topic objects, putting them into another dict
            # with a named tuple as the value
            topic_dict = {}
            for topic in self.ros_if.topics:
                tp = self.ros_if.topics[topic]
                topic_dict[topic] = TopicInfo(
                    name=tp.name,
                    fullname=tp.fullname,
                    msgtype=tp.msgtype,
                    allow_sub=tp.allow_sub,
                    allow_pub=tp.allow_pub,
                    rostype=tp.rostype,
                    rostype_name=tp.rostype_name
                )

            return topic_dict
                
        return {}

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

    def service_list(self):
        if self.ros_if:
            service_dict = {}
            for service in self.ros_if.services:
                srv = self.ros_if.services[service]
                service_dict[service] = ServiceInfo(
                    name=srv.name,
                    fullname=srv.fullname,
                    srvtype=srv.srvtype,
                    rostype_name=srv.rostype_name,
                    rostype_req=srv.rostype_req,
                    rostype_resp=srv.rostype_resp
                )

            return service_dict
                
        return {}

    def param(self, name, value=None):
        if self.ros_if and self.ros_if.get_param(name):
            if value is not None:
                self.ros_if.get_param(name).set(value)
                value = None  # consuming the message
            else:
                value = self.ros_if.get_param(name).get()
        return value

    def param_list(self):
        if self.ros_if:
            # get the dict of params, and then extract only the relevant
            # information from the topic objects, putting them into another dict
            # with a named tuple as the value
            param_dict = {}
            for param in self.ros_if.params:
                prm = self.ros_if.params[param]
                param_dict[param] = ParamInfo(
                    name=prm.name,
                    fullname=prm.fullname,
                )

            return param_dict

        return {}

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


