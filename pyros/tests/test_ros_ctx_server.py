from __future__ import absolute_import, division, print_function

import time
import nose

from pyros.client import PyrosClient
from pyros_interfaces.common.ctx_server import pyros_ctx

try:
    import pyros_interfaces_ros
except ImportError:
    raise nose.SkipTest('Importing pyros_interfaces_ros.pyros_ros FAILED ! Skipping Test...')


def testPyrosROSCtx():
    # start ros system , before PyrosROS process and Client otherwise Client assumes there is  problem ( discovery timeout )
    # we might need to load pyros_setup here...
    try:
        import pyros_utils
    except ImportError:
        # TODO : find a proper ay to log from a test like here...
        #_logger.warning("loading pyros_setup and configuring your ROS environment")
        import pyros_setup
        # This will load the pyros_setup configuration from the environment
        pyros_setup.configurable_import().configure().activate()
        import pyros_utils

    master, roscore_proc = pyros_utils.get_master(spawn=True)  # we start the master if needed

    assert master.is_online()

    with pyros_ctx(node_impl=pyros_interfaces_ros.PyrosROS) as ctx:

        assert isinstance(ctx.client, PyrosClient)

    # TODO : assert the context manager does his job ( HOW ? )

    if roscore_proc is not None:
        roscore_proc.terminate()
        while roscore_proc.is_alive():
            time.sleep(0.2)  # waiting for roscore to die


if __name__ == '__main__':

    import nose
    nose.runmodule()
