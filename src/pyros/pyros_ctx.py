from __future__ import absolute_import

from .pyros_client import PyrosClient
from .pyros_node import PyrosNode

import logging

from collections import namedtuple
from contextlib import contextmanager


# A context manager to handle rospy init and shutdown properly.
# It also creates a pipe and pass it to a node and a client.
# So stable interprocess communication can happen via the pipe
@contextmanager
#TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
def pyros_ctx(name='pyros', argv=None, anonymous=True, disable_signals=True, mock=False):
    subproc = PyrosNode(mock)
    client_conn = subproc.launch(name, argv)

    ctx = namedtuple("pyros_context", "client")
    yield ctx(client=PyrosClient(client_conn))

    subproc.shutdown()
