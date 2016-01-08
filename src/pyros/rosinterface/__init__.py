#This python package is handling all ROS related communication for rostful-node.
from __future__ import absolute_import

import os

import pyros_setup

# try to import ROS libraries.
# If failed, do a delayed import using pyros_setup
try:
    import rospy  # to make obvious that ros should be setup here ( and avoid duplicate fixes in each module )
    from .topic import TopicBack
    from .service import ServiceBack
    from .param import ParamBack
    from .ros_interface import RosInterface
    from .pyros_ros import PyrosROS
except ImportError:
    global pyros_setup
    pyros_setup = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))
    from .topic import TopicBack
    from .service import ServiceBack
    from .param import ParamBack
    from .ros_interface import RosInterface
    from .pyros_ros import PyrosROS

__all__ = [
    'ROS_emulate_setup',
    'TopicBack',
    'ServiceBack',
    'ParamBack',
    'RosInterface',
    'PyrosROS',
]
