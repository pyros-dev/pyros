from __future__ import absolute_import

import logging
import mock
from collections import namedtuple
from contextlib import contextmanager

from pyros.client import PyrosClient
import pyros.config
from pyros.interfaces.mock.pyros_mock import PyrosMock


# A context manager to handle server process launch and shutdown properly.
# It also creates a communication channel and passes it to a client.
@contextmanager
def pyros_ctx(name='pyros',
              argv=None,  # TODO : think about passing ros arguments http://wiki.ros.org/Remapping%20Arguments
              mock_client=False,
              node_impl=PyrosMock,
              pyros_config=None):

    pyros_config = pyros_config or pyros.config  # using internal config if no other config passed

    subproc = None
    ctx = namedtuple("pyros_context", "client")

    if mock_client:

        logging.warn("Setting up pyros mock client...")
        with mock.patch('pyros.client.PyrosClient', autospec=True) as client:
            yield ctx(client=client)
    else:

        logging.warn("Setting up pyros {0} node...".format(node_impl))
        subproc = node_impl(name, argv).configure(pyros_config)

        client_conn = subproc.start()

        logging.warn("Setting up pyros actual client...")
        yield ctx(client=PyrosClient(client_conn))

    if subproc is not None:
        subproc.shutdown()
