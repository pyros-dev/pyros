from __future__ import absolute_import

import ast
import json
import os
import logging
import time

# python protocol should be usable without ROS.
from .rostful_prtcl import MsgBuild, Topic, Service, ServiceList, ServiceInfo, TopicList, TopicInfo, Interaction, Interactions, InteractionInfo, Namespaces, NamespaceInfo, Rocon
from multiprocessing import Pipe
import threading
"""
Mock Interface in pure python ( No ROS needed ).
"""

class RostfulMock(object):

    def __init__(self):
        self._topic_msg = {}  # storage for the echo topic
        self._stop_event = None  # stop_event to signal the thread for soft shutdown
        self._spinner = None  # thread instance
        pass

    # These should match the design of RostfulClient and Protocol so we are consistent between pipe and python API
    def msg_build(self, name):
        msg = str()
        return msg

    # a simple echo topic
    def topic(self, name, msg_content=None):
        msg = msg_content
        if msg_content is not None:
            self._topic_msg[name] = msg_content
            msg = None  # consuming the message
        else:
            msg = self._topic_msg.get(name, None)
        return msg
        
    def topic_list(self):
        info = TopicInfo(fullname='mock fullname', allow_sub=False)
        resp_content = {'mock': info}
        return resp_content

    # a simple echo service
    def service(self, name, rqst_content=None):
        resp_content = rqst_content
        return resp_content

    def service_list(self):
        info = ServiceInfo(fullname='mock_fullname')
        resp_content = {'mock': info}
        return resp_content

    def namespaces(self):
        info = NamespaceInfo()
        resp_content = {}
        return resp_content

    def interaction(self, name):
        resp_content = Interaction(interaction=name)
        return resp_content

    def interactions(self):
        resp_content = {}
        return resp_content

    def has_rocon(self):
        return False

    def _dispatch_msg(self, pipe_conn):
        rqst = pipe_conn.recv()
        resp = rqst  # if problem we send back the exact same request message.
        # here we need to make sure we always send something back ( so the client can block safely )
        try:
            if isinstance(rqst, MsgBuild):
                resp = MsgBuild(
                    name=rqst.name,
                    msg_content=self.msg_build(rqst.name)
                )
            elif isinstance(rqst, Topic):
                resp = Topic(
                    name=rqst.name,
                    msg_content=self.topic(rqst.name, rqst.msg_content)
                )
            elif isinstance(rqst, TopicList):
                resp = TopicList(
                    name_dict=self.topic_list()
                )
            elif isinstance(rqst, Service):
                resp = Service(
                    name=rqst.name,
                    rqst_content=rqst.rqst_content,
                    resp_content=self.service(rqst.name, rqst.rqst_content)
                )
            elif isinstance(rqst, ServiceList):
                resp = ServiceList(
                    name_dict=self.service_list()
                )
            elif isinstance(rqst, Interactions):
                resp = Interactions(
                    interaction_dict=self.interactions()
                )
            elif isinstance(rqst, Namespaces):
                resp = Namespaces(
                    namespace_dict=self.namespaces()
                )
            elif isinstance(rqst, Rocon):
                resp = Rocon(
                    has_rocon=self.has_rocon()
                )
            elif isinstance(rqst, Interaction):
                resp = Interaction(
                    interaction=self.interaction(rqst.name)
                )
        finally:
            # to make sure we always return something, no matter what
            pipe_conn.send(resp)

    def spin(self, pipe_conn, check_init_fn, check_spinnable_fn):
        """

        Spinning, processing commands arriving in the queue
        :param pipe_conn :
        :param check_init_fn : needs to raise an exception to prevent starting the spin
        :param check_spinnable_fn : just returns true to break the spin
        :return:
        """
        try:
            check_init_fn()  # raise an exception to break
            # we should exit if we lose connection or stop_event disappear.
            while pipe_conn and check_spinnable_fn and check_spinnable_fn():
                try:
                    if pipe_conn.poll(0.5):
                        self._dispatch_msg(pipe_conn)
                    else:  # no data, no worries.
                        pass
                except EOFError:  # empty pipe, no worries.
                    pass
                except Exception, e:
                    raise e

        # Just in case spin() is run synchronously.
        # If run asynchronously these can only be caught in the main thread
        except KeyboardInterrupt:
            logging.debug("Keyboard Interrupt , shutting down")
        except SystemExit:
            logging.debug("System Exit , shutting down")

