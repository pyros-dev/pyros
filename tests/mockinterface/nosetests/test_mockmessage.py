from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from mockinterface import extract_msg, populate_msg, FieldTypeMismatchException
from nose.tools import assert_true, assert_false, assert_raises


def test_populate_msg_bool_true():
    msg = populate_msg(True, bool())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, bool))
    assert_true(msg)


def test_populate_msg_bool_false():
    msg = populate_msg(False, bool())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, bool))
    assert_false(msg)


def test_populate_msg_int_to_int():
    msg = populate_msg(42, int())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, int))
    assert_true(msg == 42)


def test_populate_msg_int_to_float():
    msg = populate_msg(42, float())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, float))
    assert_true(msg == 42.0)


def test_populate_msg_float():
    msg = populate_msg(3.1415, float())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, float))
    assert_true(msg == 3.1415)


def test_populate_msg_float_to_int_error():
    with assert_raises(FieldTypeMismatchException):
        populate_msg(3.1415, float())


def test_populate_msg_str_to_str():
    msg = populate_msg(r'forty two', str())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, str))
    assert_true(msg == r'forty two')


def test_populate_msg_str_to_unicode():
    msg = populate_msg(r'forty two', unicode())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, unicode))
    assert_true(msg == u'forty two')


def test_populate_msg_unicode_to_unicode():
    msg = populate_msg(u'forty two', unicode())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, unicode))
    assert_true(msg == u'forty two')


def test_populate_msg_unicode_to_str_error():
    with assert_raises(FieldTypeMismatchException):
        populate_msg(u'forty two', str())


def test_populate_msg_list():
    msg = populate_msg([False, 42, 3.1415, r'fortytwo', u'forty two'], list())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, list))
    assert_true(msg == [False, 42, 3.1415, r'fortytwo', u'forty two'])


def test_populate_msg_list_to_tuple_error():
    with assert_raises(FieldTypeMismatchException):
        populate_msg([False, 42, 3.1415, r'fortytwo', u'forty two'], tuple())


def test_populate_msg_tuple_to_tuple():
    msg = populate_msg((False, 42, 3.1415, r'fortytwo', u'forty two'), tuple())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, tuple))
    assert_true(msg == (False, 42, 3.1415, r'fortytwo', u'forty two'))


def test_populate_msg_tuple_to_list():
    msg = populate_msg((False, 42, 3.1415, r'fortytwo', u'forty two'), list())
    print "msg is of type {0}".format(type(msg))
    assert_true(isinstance(msg, list))
    assert_true(msg == [False, 42, 3.1415, r'fortytwo', u'forty two'])