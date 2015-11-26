from __future__ import absolute_import

from zmp import Node
from .mockinterface.rostful_mock import PyrosMock

try:
    import rospy
    from .rosinterface.rostful_node import PyrosROS
except ImportError, e:
    import logging
    logging.warn("Error: Could not find rospy. %s" % e)
    rospy = None

import os
import logging

from multiprocessing import Pipe, Event, Process

interface_list = {
    'ros': 'rosinterface',
    'mock': 'mockinterface',
}

#TODO : this became super simple after zmp refactoring. Can we get rid of it ?

class PyrosNode(object):
    def __init__(self, mock=False):
        self._mock = mock
        self._proc = None

    def getNodeName(self):
        return self._proc.name

    def launch(self, name='rostful_node', argv=[]):
        """
        Starts spinning in another process and returns the pipe connection to send commands to
        :param: name : the name of the node
        :param: argv : the arguments useful for the node
        :return: name of zmp.Node (enough to discover its services and start communicating).
        """

        if self._mock:
            self._proc = PyrosMock(name)
        else:
            self._proc = PyrosROS(name, argv)
        self._proc.start()
        return self._proc.name

    def shutdown(self):
        self._proc.shutdown()
