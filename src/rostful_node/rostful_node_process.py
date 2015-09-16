from __future__ import absolute_import

from .rostful_mock import RostfulMock
from .rostful_client import RostfulClient

try:
    import rospy
    from .rostful_node import RostfulNode
except ImportError, e:
    import logging
    logging.warn("Error: Could not find rospy. %s" % e)
    rospy = None

import os
import logging

from multiprocessing import Pipe, Event, Process


class RostfulNodeProcess(object):
    def __init__(self, mock=False):
        self._mock = mock
        self._proc = None
        self._stop_event = None
        self.pipe_conn = None

    def launch(self, name='rostful_node', argv=[]):
        """
        Starts spinning in another process and returns the pipe connection to send commands to
        :return: pipe end used to communicate with the thread.
        """
        self._stop_event = Event()
        self.pipe_conn, other_end = Pipe()

        # Mock functions
        def check_init():
            logging.debug("mock entering spin(), pid[%s]", os.getpid())

        def spinner( name, argv, pipe_conn, check_init, stop_event):
            node = RostfulMock()
            node.spin(
                pipe_conn,
                check_init,
                lambda: not stop_event.is_set(),  # setting the stop_event will stop the thread
            )

        # ROS Node functions
        if not self._mock:
            def check_init():
                """
                Called by the subprocess to validate the initialization of the node
                :return:
                """
                if rospy.core.is_initialized():
                    rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
                else:
                    raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")

            def spinner( name, argv, pipe_conn, check_init, stop_event):
                """
                Main subprocess code, spinning around
                :param name:
                :param argv:
                :param pipe_conn:
                :param check_init:
                :return:
                """
                # we initialize the node here, in subprocess, passing ros parameters.
                # disabling signal to avoid overriding callers behavior
                rospy.init_node(name, argv=argv, disable_signals=True)
                rospy.logwarn('rostful node started with args : %r', argv)

                node = RostfulNode()
                node.spin(
                    pipe_conn,
                    check_init,
                    lambda: not stop_event.is_set() and not rospy.core.is_shutdown()
                )
                rospy.logwarn('rostful node stopped.')

        self._proc = Process(target=spinner, args=(name, argv, self.pipe_conn, check_init, self._stop_event))
        self._proc.start()
        return other_end

    def terminate(self):
        if self._stop_event:
            self._stop_event.set()
        if self._proc:
            self._proc.join()
