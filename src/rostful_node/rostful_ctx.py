from __future__ import absolute_import

from .rostful_client import RostfulClient
from .rostful_node_process import RostfulNodeProcess

import logging

from collections import namedtuple
from contextlib import contextmanager


# A context manager to handle rospy init and shutdown properly.
# It also creates a pipe and pass it to a node and a client.
# So stable interprocess communication can happen via the pipe
@contextmanager
#TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
def rostful_ctx(name='rostful_node', argv=None, anonymous=True, disable_signals=True, mock=False):
    subproc = RostfulNodeProcess(mock)
    client_conn = subproc.launch(name, argv)

    ctx = namedtuple("rostful_context", "client")
    yield ctx(client=RostfulClient(client_conn))

    subproc.terminate()
