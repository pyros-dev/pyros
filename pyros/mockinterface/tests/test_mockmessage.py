from __future__ import absolute_import, print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from pyros.mockinterface import extract_values, populate_instance, FieldTypeMismatchException, NonexistentFieldException, StatusMsg
from nose.tools import assert_true, assert_false, assert_raises


def test_populate_msg_bool_true():
    msg = populate_instance(True, bool())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, bool))
    assert_true(msg)


def test_populate_msg_bool_false():
    msg = populate_instance(False, bool())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, bool))
    assert_false(msg)


def test_populate_msg_int_to_int():
    msg = populate_instance(42, int())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, int))
    assert_true(msg == 42)


def test_populate_msg_int_to_float():
    msg = populate_instance(42, float())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, float))
    assert_true(msg == 42.0)


def test_populate_msg_float():
    msg = populate_instance(3.1415, float())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, float))
    assert_true(msg == 3.1415)


def test_populate_msg_float_to_int_error():
    with assert_raises(FieldTypeMismatchException):
        populate_instance(3.1415, int())


def test_populate_msg_str_to_str():
    msg = populate_instance(r'forty two', str())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, str))
    assert_true(msg == r'forty two')


def test_populate_msg_str_to_unicode():
    msg = populate_instance(r'forty two', unicode())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, unicode))
    assert_true(msg == u'forty two')


def test_populate_msg_unicode_to_unicode():
    msg = populate_instance(u'forty two', unicode())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, unicode))
    assert_true(msg == u'forty two')


def test_populate_msg_unicode_to_str_error():
    with assert_raises(FieldTypeMismatchException):
        populate_instance(u'forty two', str())


def test_populate_msg_list():
    msg = populate_instance([False, 42, 3.1415, r'fortytwo', u'forty two'], list())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, list))
    assert_true(msg == [False, 42, 3.1415, r'fortytwo', u'forty two'])


def test_populate_msg_list_to_tuple_error():
    with assert_raises(FieldTypeMismatchException):
        populate_instance([False, 42, 3.1415, r'fortytwo', u'forty two'], tuple())


def test_populate_msg_tuple_to_tuple():
    msg = populate_instance((False, 42, 3.1415, r'fortytwo', u'forty two'), tuple())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, tuple))
    assert_true(msg == (False, 42, 3.1415, r'fortytwo', u'forty two'))


def test_populate_msg_tuple_to_list():
    msg = populate_instance((False, 42, 3.1415, r'fortytwo', u'forty two'), list())
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, list))
    assert_true(msg == [False, 42, 3.1415, r'fortytwo', u'forty two'])


def test_populate_msg_dict_to_status():
    msg = populate_instance(
        {"error": True, "code": 7, "message": "Actual Error"},
        StatusMsg(error=False, code=42, message="Not an error")
    )
    print("msg is of type {0}".format(type(msg)))
    assert_true(isinstance(msg, StatusMsg))
    assert_true(msg.error)
    assert_true(msg.code == 7)
    assert_true(msg.message == "Actual Error")


def test_populate_msg_dict_to_status_error():

    with assert_raises(NonexistentFieldException):
        msg = populate_instance(
            {"error": True, "code": 7, "message": "Actual Error", "non-existent": "field"},
            StatusMsg(error=False, code=42, message="Not an error")
        )


    ###### EXTRACT #######

def test_extract_msg_bool_true():
    msg = populate_instance(True, bool())
    data = extract_values(msg)
    assert_true(isinstance(data, bool))
    assert_true(data)


def test_extract_msg_bool_false():
    msg = populate_instance(False, bool())
    data = extract_values(msg)
    assert_true(isinstance(data, bool))
    assert_false(data)


def test_extract_msg_int_to_int():
    msg = populate_instance(42, int())
    data = extract_values(msg)
    assert_true(isinstance(data, int))
    assert_true(data == 42)


def test_extract_msg_int_to_float():
    msg = populate_instance(42, float())
    data = extract_values(msg)
    assert_true(isinstance(data, float))
    assert_true(data == 42.0)


def test_extract_msg_float():
    msg = populate_instance(3.1415, float())
    data = extract_values(msg)
    assert_true(isinstance(data, float))
    assert_true(data == 3.1415)


def test_extract_msg_str_to_str():
    msg = populate_instance(r'forty two', str())
    data = extract_values(msg)
    assert_true(isinstance(data, str))
    assert_true(data == r'forty two')


def test_extract_msg_str_to_unicode():
    msg = populate_instance(r'forty two', unicode())
    data = extract_values(msg)
    assert_true(isinstance(data, unicode))
    assert_true(data == u'forty two')


def test_extract_msg_unicode_to_unicode():
    msg = populate_instance(u'forty two', unicode())
    data = extract_values(msg)
    assert_true(isinstance(data, unicode))
    assert_true(data == u'forty two')


def test_extract_msg_list():
    msg = populate_instance([False, 42, 3.1415, r'fortytwo', u'forty two'], list())
    data = extract_values(msg)
    assert_true(isinstance(data, list))
    assert_true(data == [False, 42, 3.1415, r'fortytwo', u'forty two'])


def test_extract_msg_tuple_to_tuple():
    msg = populate_instance((False, 42, 3.1415, r'fortytwo', u'forty two'), tuple())
    data = extract_values(msg)
    assert_true(isinstance(data, tuple))
    assert_true(data == (False, 42, 3.1415, r'fortytwo', u'forty two'))


def test_extract_msg_tuple_to_list():
    msg = populate_instance((False, 42, 3.1415, r'fortytwo', u'forty two'), list())
    data = extract_values(msg)
    assert_true(isinstance(msg, list))
    assert_true(msg == [False, 42, 3.1415, r'fortytwo', u'forty two'])


def test_extract_msg_dict_to_status():
    msg = populate_instance(
        {"error": True, "code": 7, "message": "Actual Error"},
        StatusMsg(error=False, code=42, message="Not an error")
    )
    data = extract_values(msg)
    assert_true(isinstance(data, StatusMsg))
    assert_true(data.error)
    assert_true(data.code == 7)
    assert_true(data.message == "Actual Error")

if __name__ == '__main__':

    import nose
    nose.runmodule()
