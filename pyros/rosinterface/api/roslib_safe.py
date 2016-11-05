from __future__ import absolute_import
from __future__ import print_function

import socket
import time
import roslib


# We wrap rospy function into safeguards for socket error
# since the master seems to be quite sensitive to Network health.
# TODO : refine which ones need which exception handling...