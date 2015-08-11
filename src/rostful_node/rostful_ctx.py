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

import logging

from multiprocessing import Pipe
from collections import namedtuple
from contextlib import contextmanager

# A context manager to handle rospy init and shutdown properly.
# It also creates a pipe and pass it to a node and a client.
# So stable interprocess communication can happen via the pipe
@contextmanager
#TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
def RostfulCtx(name='rostful_node', argv=None, anonymous=True, disable_signals=True, mock=False):
    if rospy and not mock:
        #we initialize the node here, passing ros parameters.
        #disabling signal to avoid overriding callers behavior
        rospy.init_node(name, argv=argv, anonymous=anonymous, disable_signals=disable_signals)
        rospy.logwarn('rostful node started with args : %r', argv)

        node = RostfulNode()
        client_conn = node.async_spin()
        ctx = namedtuple("rostful_context", "node client")
        yield ctx(node=node, client=RostfulClient(client_conn))

        node.async_stop()
        rospy.logwarn('rostful node stopped')
        # rospy.signal_shutdown('Closing')  # probably useless now ?

    else:
        logging.warn('rostful mock node started with args : %r', argv)

        # no ROS installed : pure python mock.
        mock = RostfulMock()
        client_conn = mock.async_spin()
        ctx = namedtuple("rostful_context", "node client")
        yield ctx(node=mock, client=RostfulClient(client_conn))

        mock.async_stop()
        logging.warn('rostful mock node stopped')


