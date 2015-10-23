from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from mockinterface import extract_msg, populate_msg

type_map = {
   "bool":    ["bool"],
   "int":     ["int"],
   "float":   ["float"],
   "str":     ["str"],
   "unicode": ["unicode"],  # also convert str to unicode to be ready for python 3 default behavior ?
   "list":    ["list"],
   "tuple":   ["tuple"],
}

def test_populate_msg_bool():
    msg = populate_msg(True, bool())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, bool)

def test_populate_msg_int():
    msg = populate_msg(42, int())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, int)

def test_populate_msg_float():
    msg = populate_msg(3.1415, float())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, float)

def test_populate_msg_str():
    msg = populate_msg(r'forty two', str())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, str)

def test_populate_msg_unicode():
    msg = populate_msg(u'forty two', unicode())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, unicode)

def test_populate_msg_list():
    msg = populate_msg([False, 42, 3.1415, r'fortytwo', u'forty two'], list())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, list)

def test_populate_msg_tuple():
    msg = populate_msg((False, 42, 3.1415, r'fortytwo', u'forty two'), tuple())
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, tuple)
