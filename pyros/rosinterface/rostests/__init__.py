# Because rospy :
# - expects OS state to be very special
# - modify global python interpreter state when using node_init ( which is required to receive message on topics )
# => Communication features are only testable with rostest.
# However, running them one by one with nosetest works (One run, one test module, one process).
# This allows easy debugging ( even during start / stop sequences )
# Therefore, tests are executable so that nosetests doesn't auto detect them.

import time


class Timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after

# reused fixtures:
from .testRosInterface import TestRosInterface
