#!/usr/bin/env python
from __future__ import absolute_import

# ROS SETUP if needed
try:
    import rospy
    import rosgraph
    import roslaunch
except ImportError, ie:
    import os
    import sys
    import pprint
    import subprocess

    command = ['bash', '-c', 'source /opt/ros/indigo/setup.bash && env']

    proc = subprocess.Popen(command, stdout=subprocess.PIPE)

    for line in proc.stdout:
      (key, _, value) = line.partition("=")
      os.environ[key] = value.rstrip()
      if key == 'PYTHONPATH':
          for newp in [p for p in value.split(':') if p not in sys.path]:
              sys.path.append(newp)

    proc.communicate()

    import rospy
    import rosgraph
    import roslaunch

import multiprocessing
import time
import cProfile
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

rosn = PyrosROS(dynamic_reconfigure=False)


def update_loop():
    count = 255
    while count > 0:
        rosn.update()
        time.sleep(0.1)
        count -= 1

update_loop()

cProfile.run('rosn.update()')

rospy.signal_shutdown('test complete')

if roscore_process is not None:
    roscore_process.terminate()  # make sure everything is stopped