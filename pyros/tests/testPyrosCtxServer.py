from __future__ import absolute_import

import sys
import os

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec
import time

from pyros.pyros_ctx_server import pyros_ctx
from pyros.pyros_client import PyrosClient


def testPyrosMockCtx():
    with pyros_ctx(mock_node=True) as ctx:
        assert isinstance(ctx.client, PyrosClient)

    # TODO : assert the context manager does his job ( HOW ? )


def testPyrosROSCtx():
    # start ros system , before PyrosROS process and Client otherwise Client assumes there is  problem ( discovery timeout )
    import pyros_setup
    master, roscore_proc = pyros_setup.get_master(spawn=True)  # we start the master if needed

    assert master.is_online()

    with pyros_ctx(mock_node=False) as ctx:

        assert isinstance(ctx.client, PyrosClient)

    # TODO : assert the context manager does his job ( HOW ? )

    if roscore_proc is not None:
        roscore_proc.terminate()
        while roscore_proc.is_alive():
            time.sleep(0.2)  # waiting for roscore to die


if __name__ == '__main__':

    import nose
    nose.runmodule()
