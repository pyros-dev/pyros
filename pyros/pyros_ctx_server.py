from __future__ import absolute_import

import logging
import mock
from collections import namedtuple
from contextlib import contextmanager

from . import pyros_client, config
from .pyros_mock import PyrosMock
from .pyros_ros import PyrosROS


# A context manager to handle server process launch and shutdown properly.
# It also creates a communication channel and passes it to a client.
@contextmanager
def pyros_ctx(name='pyros',
              argv=None,  # TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
              mock_client=False,
              mock_node=False,
              pyros_config=None):

    pyros_config = pyros_config or config  # using internal config if no other config passed

    subproc = None
    if not mock_client:
        if mock_node:
            logging.warn("Setting up pyros mock node...")
            subproc = PyrosMock(name)
        else:
            logging.warn("Setting up pyros ROS node...")
            subproc = PyrosROS(name, argv).configure(pyros_config)

        client_conn = subproc.start()

    ctx = namedtuple("pyros_context", "client")
    if mock_client:

        logging.warn("Setting up pyros mock client...")
        with mock.patch(pyros_client.__name__ + '.PyrosClient', autospec=True) as client:
            yield ctx(client=client)
    else:

        logging.warn("Setting up pyros actual client...")
        yield ctx(client=pyros_client.PyrosClient(client_conn))

    if subproc is not None:
        subproc.shutdown()
