from __future__ import absolute_import
from __future__ import print_function

import socket
import time
import rosnode
from . import rospy_safe as rospy

# We wrap rosnode function into safeguards for socket error
# since the master seems to be quite sensitive to Network health.
# TODO : refine which ones need which exception handling...

retry_timeout = 1  # number of seconds to wait before retrying


def get_api_uri(master, caller_id):
    res = None
    while res is None:
        try:
            res = rosnode.get_api_uri(master, caller_id)
        except (socket.error, socket.herror, socket.gaierror) as e:
            rospy.logerr(
                "Pyros : got socket error calling get_api_uri({master}, {caller_id}). Retrying...".format(**locals()))
            time.sleep(retry_timeout)
        else:
            return res