from __future__ import absolute_import

import ast
import json
import os
import logging
import time

# python protocol should be usable without ROS.
from .rostful_prtcl import MsgBuild, Topic, Service

from multiprocessing import Pipe
import threading
"""
Mock Interface in pure python ( No ROS needed ).
"""

class RostfulMock(object):

    def __init__(self, pipe_conn=None):
        self._topic_msg = {}  # storage for the echo topic
        self.pipe_conn = pipe_conn
        pass

    # These should match the design of RostfulClient and Protocol so we are consistent between pipe and python API
    def msg_build(self, connec_name):
        msg = str()
        return msg

    # a simple echo topic
    def topic(self, name, msg_content=None):
        msg = msg_content
        if msg_content:
            self._topic_msg[name] = msg_content
            msg = None  # consuming the message
        else:
            msg = self._topic_msg.get(name, None)
        return msg

    # a simple echo service
    def service(self, name, rqst_content=None):
        resp_content = rqst_content
        return resp_content

    def _process_pipe_msg(self):
        rqst = self.pipe_conn.recv()
        resp = rqst  # if problem we send back the exact same request message.
        # here we need to make sure we always send something back ( so the client can block safely )
        try:
            if isinstance(rqst, MsgBuild):
                resp = MsgBuild(
                    connec_name=rqst.connec_name,
                    msg_value=self.msg_build(rqst.connec_name)
                )

            elif isinstance(rqst, Topic):
                resp = Topic(
                    name=rqst.name,
                    msg_value=self.topic(rqst.name, rqst.msg_value)
                )

            elif isinstance(rqst, Service):
                resp = Service(
                    name=rqst.name,
                    rqst_value=rqst.rqst_value,
                    resp_value=self.service(rqst.name, rqst.rqst_value)
                )

        finally:
            # to make sure we always return something, no matter what
            self.pipe_conn.send(resp)

    def spin(self, stop_event):
        """
        Spinning, processing commands arriving in the queue
        :return:
        """
        # check if INITED ( using Context or not )

        logging.debug("mock entering spin(), pid[%s]", os.getpid())
        try:
            while not stop_event.is_set():
                try:
                    if self.pipe_conn:
                        if self.pipe_conn.poll(0.5):
                            self._process_pipe_msg()
                        else:  # no data, no worries.
                            pass
                    else:
                        time.sleep(0.5)
                except EOFError:  # empty pipe, no worries.
                    pass
                except Exception, e:
                    raise e

        # Just in case spin() is run synchronously.
        # If run asynchronously these can only be caught in the main thread
        except KeyboardInterrupt:
            logging.debug("Keyboard Interrupt , shutting down")
            # trigger shutdown
        except SystemExit:
            logging.debug("System Exit , shutting down")

    def async_spin(self):

        self._stop_event = threading.Event()

        self._spinner = threading.Thread(target=self.spin, args=(self._stop_event,))
        self._spinner.start()

    # parent thread needs to call this to terminate the thread gracefully
    def async_stop(self):
        self._stop_event.set()
        self._spinner.join()

    def __del__(self):
        # to make sure we stop the thread when we dont need it anymore.
        self.async_stop()
