from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

# Unit test import
from pyros.rosinterface import message_conversion as msgconv
# ROS imports should now work from ROS or from python ( even without ROS env setup)
import rospy

# useful test tools
import nose
from nose.tools import assert_equal, assert_true, assert_false

# Test all standard message
import std_msgs.msg as std_msgs


# TODO : change this customJSON -> ROS conversion by JSON -ujson-> py -rospy-> ROS
# TODO : achieve SYMMETRY : what we put in == what we get out ( no "data" field added )
def test_String_default():
    msg = std_msgs.String()
    val = msgconv.extract_values(msg)
    assert_equal(val["data"], str())  # "data" : should not appear here


def test_String_custom():
    msg = std_msgs.String("teststr")
    msgconv.populate_instance({"data": "teststr2"}, msg)  # we shouldnt need "data" here
    val = msgconv.extract_values(msg)
    assert_equal(val["data"], "teststr2")


if __name__ == '__main__':
    nose.runmodule()
