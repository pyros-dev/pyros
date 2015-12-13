from __future__ import absolute_import

import logging

from .mockinterface.mocknode import PyrosMock
from .pyros_client import PyrosClient

try:
    # this will import rosinterface and if needed simulate ROS setup
    from .rosinterface.pyros_ros import PyrosROS
except ImportError, e:
    import logging
    logging.warn("Error: Could not import PyrosROS from .rosinterface.pyros_ros {}".format(e))


from collections import namedtuple
from contextlib import contextmanager


# A context manager to handle server process launch and shutdown properly.
# It also creates a communication channel and passes it to a client.
@contextmanager
# TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
def pyros_ctx(name='pyros', argv=None, anonymous=True, disable_signals=True, mock=False):

    if mock:
        subproc = PyrosMock(name)
    else:
        subproc = PyrosROS(name, argv)
    client_conn = subproc.start()

    ctx = namedtuple("pyros_context", "client")
    yield ctx(client=PyrosClient(client_conn))

    subproc.shutdown()
