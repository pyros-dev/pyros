from __future__ import absolute_import, division, print_function

import time
import pytest

from pyros.client import PyrosClient
from pyros.server.ctx_server import pyros_ctx

pyros_interfaces_ros = pytest.importorskip("pyros_interfaces_ros", minversion="0.4")


def testPyrosROSCtx():
    # start ros system , before PyrosROS process and Client otherwise Client assumes there is  problem ( discovery timeout )
    # we might need to load pyros_setup here...
    try:
        import pyros_utils
    except ImportError:
        # TODO : find a proper way to log from a test like here...
        try :
            #_logger.warning("loading pyros_setup and configuring your ROS environment")
            import pyros_setup
            # This will load the pyros_setup configuration from the environment
            pyros_setup.configurable_import().configure().activate()
            import pyros_utils
        except ImportError:
            # This is expected when testing pyros by itself
            raise nose.SkipTest("pyros_utils could not be imported, and trying to import pyros_setup for dynamic ros setup failed.")

    master, roscore_proc = pyros_utils.get_master(spawn=True)  # we start the master if needed

    assert master.is_online()

    with pyros_ctx(node_impl=pyros_interfaces_ros.PyrosROS) as ctx:

        assert isinstance(ctx.client, PyrosClient)

    # TODO : assert the context manager does his job ( HOW ? )

    if roscore_proc is not None:
        roscore_proc.terminate()
        while roscore_proc.is_alive():
            time.sleep(0.2)  # waiting for roscore to die


# Just in case we run this directly
if __name__ == '__main__':
    import pytest
    pytest.main([
        '-s', __file__,
])
