from __future__ import absolute_import

import os
import sys
import pickle

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

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


def test_msg_exception_pickle():
    exc = msgconv.NonexistentFieldException("message type", ["field1", "field2"])

    pbuf = pickle.dumps(exc)
    pexc = pickle.loads(pbuf)

    assert_equal(pexc.basetype, "message type")
    assert_equal(len(pexc.fields), 2)
    assert_true("field1" in pexc.fields)
    assert_true("field2" in pexc.fields)

# TODO : assert both "string" and "std_msgs/String" are convertible from&to "str" and "unicode"


#TODO : assert exception are being thrown

if __name__ == '__main__':
    nose.runmodule()
