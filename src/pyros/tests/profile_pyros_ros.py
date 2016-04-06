#!/usr/bin/env python
from __future__ import absolute_import

# ROS SETUP if needed

import multiprocessing
import time
import cProfile
try:
    import rospy
    import rosgraph
    import roslaunch
    from pyros.rosinterface import PyrosROS
except ImportError as exc:
    import os
    import pyros.rosinterface
    import sys
    sys.modules["pyros.rosinterface"] = pyros.rosinterface.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..'))
    import rospy
    import rosgraph
    import roslaunch
    from pyros.rosinterface import PyrosROS

roscore_process = None

if not rosgraph.masterapi.is_online():
    # Trying to solve this : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
    def ros_core_launch():
        roslaunch.main(['roscore', '--core'])  # same as rostest_main implementation

    roscore_process = multiprocessing.Process(target=ros_core_launch)
    roscore_process.start()

# Start roslaunch
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

rosn = PyrosROS()


def update_loop():
    count = 255
    start = time.time()
    while count > 0:
        # time is ticking
        now = time.time()
        timedelta = now - start
        start = now

        rosn.update(timedelta)

        count -= 1

cProfile.run('update_loop()')

rospy.signal_shutdown('test complete')

if roscore_process is not None:
    roscore_process.terminate()  # make sure everything is stopped