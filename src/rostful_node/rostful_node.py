from __future__ import absolute_import

import rospy

from .ros_interface import RosInterface
try:
    from .rocon_interface import RoconInterface
    _ROCON_AVAILABLE = True
except ImportError, e:
    import logging
    logging.warn("Error: could not import RoconInterface - disabling. %s" % e)
    _ROCON_AVAILABLE = False
    
from .rostful_prtcl import MsgBuild, Topic, Service
from .rostful_mock import RostfulMock

from dynamic_reconfigure.server import Server
from rostful_node.cfg import RostfulNodeConfig
import ast
import json
import os

import rostful_node.srv as srv
from rosinterface import message_conversion as msgconv
from rosinterface.action import ActionBack

from multiprocessing import Pipe
import threading
"""
Interface with ROS.
"""

class RostfulNode(RostfulMock):
    def __init__(self):
        super(RostfulNode, self).__init__()
        enable_rocon = rospy.get_param('~enable_rocon', False)
        self.enable_rocon = enable_rocon or (
            (len(ast.literal_eval(rospy.get_param('~rapps_namespaces', "[]"))) > 0)
            or (len(ast.literal_eval(rospy.get_param('~interactions', "[]"))) > 0)
        )

        self.ros_if = RosInterface()

        if _ROCON_AVAILABLE and self.enable_rocon:
            self.rocon_if = RoconInterface(self.ros_if)
            pass
        else:
            self.rocon_if = None

        # Create a dynamic reconfigure server.
        self.server = Server(RostfulNodeConfig, self.reconfigure)

        ##############################################################################################
        #### Helpers in case we need to listen to someone talking from a different process
        # Needed because of limitation in rospy that we cannot publish on topic from different process
        # Note : It should be working fine for services however
        # TODO : change this to use pure python code to avoid confusion ( for ex. using multiprocessing lib )
        ##############################################################################################
        def inject_topic(req):  # Keep this minimal
            rospy.logwarn("""Requesting Topic {topic} Injection: \n{data} """.format(
                topic=req.topic_name,
                data=req.data_json
            ))

            #normalizing names... ( somewhere else ?)
            #topic_name = unicodedata.normalize('NFKD', req.topic_name).encode('ascii', 'ignore')
            #topic is raw str
            if req.topic_name[0] == '/':
                req.topic_name = req.topic_name[1:]

            res = False
            if self.ros_if and req.topic_name in self.ros_if.topics:
                topic = self.ros_if.topics[req.topic_name]

                input_msg_type = topic.rostype
                input_msg = input_msg_type()

                input_data = json.loads(req.data_json)
                input_data.pop('_format', None)  # to follow REST interface design ( maybe shouldnt be here )
                msgconv.populate_instance(input_data, input_msg)

                topic.publish(input_msg)
                res = True

            return srv.InjectTopicResponse(res)

        def extract_topic(req):  # Keep this minimal
            rospy.logwarn("""Requesting Topic {topic} Extraction: """.format(
                topic=req.topic_name
            ))
            #normalizing names... ( somewhere else ?)
            #topic_name = unicodedata.normalize('NFKD', req.topic_name).encode('ascii', 'ignore')
            #topic is raw str
            if req.topic_name[0] == '/':
                req.topic_name = req.topic_name[1:]

            res = False
            output_data = '{}'
            if self.ros_if and req.topic_name in self.ros_if.topics:
                topic = self.ros_if.topics[req.topic_name]
                msg = topic.get()

                output_data = msgconv.extract_values(msg) if msg is not None else None
                output_data['_format'] = 'ros'  # to follow existing REST behavior
                output_data = json.dumps(output_data)
                res = True

            return srv.ExtractTopicResponse(res, output_data)

        def call_service(req):  # Keep this minimal
            rospy.logwarn("""Requesting Service {service} Call: """.format(
                service=req.service_name
            ))
            #normalizing names... ( somewhere else ?)
            #service_name = unicodedata.normalize('NFKD', req.service_name).encode('ascii', 'ignore')
            #service is raw str
            if req.service_name[0] == '/':
                req.service_name = req.service_name[1:]

            output_data = '{}'
            if self.ros_if and req.service_name in self.ros_if.services:
                service = self.ros_if.services[req.service_name]
                input_msg_type = service.rostype_req
                input_msg = input_msg_type()

                input_data = json.loads(req.data_json)
                input_data.pop('_format', None)  # to follow REST interface design ( maybe shouldnt be here )
                msgconv.populate_instance(input_data, input_msg)

                ret_msg = service.call(input_msg)

                output_data = msgconv.extract_values(ret_msg)
                output_data['_format'] = 'ros'  # to follow existing REST behavior
                output_data = json.dumps(output_data)

            return srv.CallServiceResponse(output_data)

        def start_action(req):  # Keep this minimal
            rospy.logwarn("""Requesting Action {action} Start: \n{data} """.format(
                action=req.action_name,
                data=req.data_json
            ))

            #normalizing names... ( somewhere else ?)
            #action_name = unicodedata.normalize('NFKD', req.action_name).encode('ascii', 'ignore')
            #action is raw str
            if req.action_name[0] == '/':
                req.action_name = req.action_name[1:]

            res = False
            if self.ros_if and req.action_name in self.ros_if.actions:
                action = self.ros_if.actions[req.action_name]

                goal_msg_type = action.get_msg_type(ActionBack.GOAL_SUFFIX)
                goal_msg = goal_msg_type()

                input_data = json.loads(req.data_json)
                input_data.pop('_format', None)  # to follow REST interface design ( maybe shouldnt be here )
                msgconv.populate_instance(input_data, goal_msg)

                action.publish_goal(goal_msg)
                res = True

            return srv.StartActionResponse(res)

        def cancel_action(req):  # Keep this minimal
            rospy.logwarn("""Requesting Action {action} Cancel: \n{data} """.format(
                action=req.action_name,
                data=req.data_json
            ))

            #normalizing names... ( somewhere else ?)
            #action_name = unicodedata.normalize('NFKD', req.action_name).encode('ascii', 'ignore')
            #action is raw str
            if req.action_name[0] == '/':
                req.action_name = req.action_name[1:]

            res = False
            if self.ros_if and req.action_name in self.ros_if.actions:
                action = self.ros_if.actions[req.action_name]

                cancel_msg_type = action.get_msg_type(ActionBack.CANCEL_SUFFIX)
                cancel_msg = cancel_msg_type()

                cancel_data = json.loads(req.data_json)
                cancel_data.pop('_format', None)  # to follow REST interface design ( maybe shouldnt be here )
                msgconv.populate_instance(cancel_data, cancel_msg)

                action.publish_cancel(cancel_msg)
                res = True

            return srv.CancelActionResponse(res)

        def status_action(req):  # Keep this minimal
            rospy.logwarn("""Requesting Action {action} Status""".format(
                action=req.action_name
            ))

            #normalizing names... ( somewhere else ?)
            #action_name = unicodedata.normalize('NFKD', req.action_name).encode('ascii', 'ignore')
            #action is raw str
            if req.action_name[0] == '/':
                req.action_name = req.action_name[1:]

            res = False
            if self.ros_if and req.action_name in self.ros_if.actions:
                action = self.ros_if.actions[req.action_name]

                msg = action.get_status()

                if msg is not None:
                    output_data = msgconv.extract_values(msg)
                    output_data['_format'] = 'ros'  # to follow existing REST behavior
                else:
                    output_data = None
                output_data = json.dumps(output_data)

            return srv.StatusActionResponse(output_data)

        def feedback_action(req):  # Keep this minimal
            rospy.logwarn("""Requesting Action {action} Feedback""".format(
                action=req.action_name
            ))

            #normalizing names... ( somewhere else ?)
            #action_name = unicodedata.normalize('NFKD', req.action_name).encode('ascii', 'ignore')
            #action is raw str
            if req.action_name[0] == '/':
                req.action_name = req.action_name[1:]

            res = False
            if self.ros_if and req.action_name in self.ros_if.actions:
                action = self.ros_if.actions[req.action_name]

                msg = action.get_feedback()

                if msg is not None:
                    output_data = msgconv.extract_values(msg)
                    output_data['_format'] = 'ros'  # to follow existing REST behavior
                else:
                    output_data = None
                output_data = json.dumps(output_data)

            return srv.FeedbackActionResponse(output_data)

        def result_action(req):  # Keep this minimal
            rospy.logwarn("""Requesting Action {action} Result""".format(
                action=req.action_name
            ))

            #normalizing names... ( somewhere else ?)
            #action_name = unicodedata.normalize('NFKD', req.action_name).encode('ascii', 'ignore')
            #action is raw str
            if req.action_name[0] == '/':
                req.action_name = req.action_name[1:]

            res = False
            if self.ros_if and req.action_name in self.ros_if.actions:
                action = self.ros_if.actions[req.action_name]

                msg = action.get_result()

                output_data = msgconv.extract_values(msg) if msg is not None else None
                output_data['_format'] = 'ros'  # to follow existing REST behavior
                output_data = json.dumps(output_data)

            return srv.ResultActionResponse(output_data)

        def start_rapp(req):  # Keep this minimal
            rospy.logwarn("""Requesting Rapp Start {rapp}: """.format(
                rapp=req.rapp_name
            ))
            #normalizing names... ( somewhere else ?)
            #service_name = unicodedata.normalize('NFKD', req.service_name).encode('ascii', 'ignore')
            #service is raw str
            if req.rapp_name[0] == '/':
                req.rapp_name = req.rapp_name[1:]

            if self.rocon_if :
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

        self.ActionStartService = rospy.Service('~start_action', srv.StartAction, start_action)
        self.ActionCancelService = rospy.Service('~cancel_action', srv.CancelAction, cancel_action)
        self.ActionStatusService = rospy.Service('~status_action', srv.StatusAction, status_action)
        self.ActionFeedbackService = rospy.Service('~feedback_action', srv.FeedbackAction, feedback_action)
        self.ActionResultService = rospy.Service('~result_action', srv.ResultAction, result_action)
        self.RappStartService = rospy.Service('~start_rapp', srv.StartRapp, start_rapp)
        self.RappStopService = rospy.Service('~stop_rapp', srv.StopRapp, stop_rapp)

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
        msg = self.msg_build(name)
        if self.ros_if and self.ros_if.get_topic(name):
            if msg_content:
                msgconv.populate_instance(msg_content, msg)
                self.ros_if.get_topic(name).publish(msg)
                msg = None  # consuming the message
            else:
                res = self.ros_if.get_topic(name).get(consume=False)
                msg = msgconv.extract_values(res) if res else res
        return msg

    def service(self, name, rqst_content=None):
        rqst = self.msg_build(name)
        msgconv.populate_instance(rqst_content, rqst)
        resp_content = None
        if self.ros_if and self.ros_if.get_service(name):
            resp = self.ros_if.get_service(name).call(rqst)
            resp_content = msgconv.extract_values(resp)
        return resp_content
    ###

    def async_spin(self):
        """
        Starts spinning in another thread and returns the pipe connection to send commands to
        :return: pipe end used to communicate with the thread.
        """
        self._stop_event = threading.Event()
        pipe_conn, other_end = Pipe()

        # TODO : check about synchronization to avoid concurrency on pip write/read ( in case of multiple clients for example )

        def check_init():
            if rospy.core.is_initialized():
                rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
            else:
                raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")

        self._spinner = threading.Thread(target=self.spin, args=(
            pipe_conn,
            check_init,
            lambda: not self._stop_event.is_set() and not rospy.core.is_shutdown(),
        ))
        self._spinner.start()
        return other_end

    # parent thread needs to call this to terminate the thread gracefully
    def async_stop(self):
        if self._stop_event:
            self._stop_event.set()
        if self._spinner:
            self._spinner.join()

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        rospy.logwarn("""Reconfigure Request: \renable_rocon : {enable_rocon}""".format(**config))
        self.enable_rocon = config["enable_rocon"] or (
            len(ast.literal_eval(config["rapps_namespaces"])) > 0
            or len(ast.literal_eval(config["interactions"])) > 0
        )

        if not self.rocon_if and self.enable_rocon:
            self.rocon_if = RoconInterface(self.ros_if)

        config = self.ros_if.reconfigure(config, level)

        if self.rocon_if:
            config = self.rocon_if.reconfigure(config, level)

        return config


