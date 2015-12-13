from __future__ import absolute_import

import os
import sys

# to be able to run from source and access srv and other catkin generated classes in devel space
print sys.path
src_py_pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if os.path.exists(src_py_pkg):
    sys.path.append(src_py_pkg)

from pyros.rosinterface import ros_setup

try:
    import roslib
    import rospy
except ImportError:
    # emulating ros setup when needed here allow us to launch nodes and debug them from IDE.
    ros_setup.ROS_emulate_setup()
    import roslib
    import rospy


class TestArgumentNotFound(Exception):
    pass
