from __future__ import absolute_import

import rospy

from .ros_interface import RosInterface

_ROCON_IF = False
try :
    from roconinterface.rocon_interface import RoconInterface
    _ROCON_IF = True
except Exception, e:
    rospy.logwarn('Missing rocon interface. Rocon features disabled')


from dynamic_reconfigure.server import Server
from rostful_node.cfg import RostfulNodeConfig
import ast
import json

import rostful_node.srv as srv
from rosinterface import message_conversion as msgconv
from rosinterface.action import ActionBack
"""
Interface with ROS.
"""

class RostfulNodeImpl(object):
    def __init__(self):

        enable_rocon = rospy.get_param('~enable_rocon', False)
        self.enable_rocon = enable_rocon or (
            (len(ast.literal_eval(rospy.get_param('~rapps_namespaces', "[]"))) > 0)
            or (len(ast.literal_eval(rospy.get_param('~interactions', "[]"))) > 0)
        )

        self.ros_if = RosInterface()

        if _ROCON_IF and self.enable_rocon:
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


        self.TopicInjectService = rospy.Service('inject_topic', srv.InjectTopic, inject_topic)
        self.TopicExtractService = rospy.Service('extract_topic', srv.ExtractTopic, extract_topic)
        self.ServiceCallService = rospy.Service('call_service', srv.CallService, call_service)
        self.ActionStartService = rospy.Service('start_action', srv.StartAction, start_action)
        self.ActionCancelService = rospy.Service('cancel_action', srv.CancelAction, cancel_action)
        self.ActionStatusService = rospy.Service('status_action', srv.StatusAction, status_action)
        self.ActionFeedbackService = rospy.Service('feedback_action', srv.FeedbackAction, feedback_action)
        self.ActionResultService = rospy.Service('result_action', srv.ResultAction, result_action)

        ##############################################################################################

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        rospy.logwarn("""Reconfigure Request: \renable_rocon : {enable_rocon}""".format(**config))
        self.enable_rocon = config["enable_rocon"] or (
            len(ast.literal_eval(config["rapps_namespaces"])) > 0
            or len(ast.literal_eval(config["interactions"])) > 0
        )

        if _ROCON_IF and not self.rocon_if and self.enable_rocon:
            self.rocon_if = RoconInterface(self.ros_if)

        config = self.ros_if.reconfigure(config, level)

        if _ROCON_IF and self.rocon_if:
            config = self.rocon_if.reconfigure(config, level)

        return config

from contextlib import contextmanager
# A context manager to handle rospy init and shutdown properly.
@contextmanager
#TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
def RostfulNode(name='rostful_node', argv=None, anonymous=True, disable_signals=True):
    #we initialize the node here, passing ros parameters.
    #disabling signal to avoid overriding callers behavior
    rospy.init_node(name, argv=argv, anonymous=anonymous, disable_signals=disable_signals)
    rospy.logwarn('rostful node started with args : %r', argv)

    yield RostfulNodeImpl()
    rospy.logwarn('rostful node stopped')
    rospy.signal_shutdown('Closing')

