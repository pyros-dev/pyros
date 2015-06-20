from __future__ import absolute_import

import rospy

from .ros_interface import RosInterface

_ROCON_IF = False
try :
    from roconinterface.rocon_interface import RoconInterface
    _ROCON_IF = True
except Exception, e:
    rospy.logwarn('Missing rocon interface. Rocon features disabled')


from dynamic_reconfigure.server import Server
from rostful_node.cfg import RostfulNodeConfig
import ast
import signal

from flask import request

"""
Interface with ROS.
No inheritance to make sure destructor is called properly.
"""
from contextlib import contextmanager

#@contextmanager
#def tag(name):
#    print "<%s>" % name
#    yield
#    print "</%s>" % name

#>>> with tag("h1"):
#...    print "foo"

# A context manager to handle rospy init and shutdown properly.
@contextmanager
def RostfulNode(ros_args):
    #we initialize the node here, passing ros parameters.
    #disabling signal to avoid overriding callers behavior
    rospy.init_node('rostful', argv=ros_args, anonymous=True, disable_signals=True)
    rospy.logwarn('rostful node started with args : %r', ros_args)

    class RostfulNodeImpl(object):
        def __init__(self):

            enable_rocon = rospy.get_param('~enable_rocon', False)
            self.enable_rocon = enable_rocon or (
                (len(ast.literal_eval(rospy.get_param('~rapps_namespaces', "[]"))) > 0)
                or (len(ast.literal_eval(rospy.get_param('~interactions', "[]"))) > 0)
            )

            self.ros_if = RosInterface()

            if _ROCON_IF and self.enable_rocon:
                self.rocon_if = RoconInterface(self.ros_if)
                pass
            else:
                self.rocon_if = None

            # Create a dynamic reconfigure server.
            self.server = Server(RostfulNodeConfig, self.reconfigure)

        # Create a callback function for the dynamic reconfigure server.
        def reconfigure(self, config, level):
            rospy.logwarn("""Reconfigure Request: \renable_rocon : {enable_rocon}""".format(**config))
            self.enable_rocon = config["enable_rocon"] or (
                len(ast.literal_eval(config["rapps_namespaces"])) > 0
                or len(ast.literal_eval(config["interactions"])) > 0
            )

            if _ROCON_IF and not self.rocon_if and self.enable_rocon:
                self.rocon_if = RoconInterface(self.ros_if)

            config = self.ros_if.reconfigure(config, level)

            if _ROCON_IF and self.rocon_if:
                config = self.rocon_if.reconfigure(config, level)

            return config

    yield RostfulNodeImpl()
    rospy.logwarn('rostful node stopped')
    rospy.signal_shutdown('Closing')

