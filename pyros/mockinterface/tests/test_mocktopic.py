from __future__ import absolute_import, print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from pyros.mockinterface import extract_values, populate_instance, FieldTypeMismatchException, NonexistentFieldException, StatusMsg
from pyros.mockinterface import statusecho_topic, MockTopic
from nose.tools import assert_true, assert_false, assert_raises


def test_echo_status():
    msg = populate_instance({
        'error': 'BAD_ERROR',
        'code': 7,
        'message': 'A bad Error happened'
    }, StatusMsg('ERROR', 42, 'details'))
    assert_true(isinstance(msg, StatusMsg))
    echo_topic = MockTopic('random_topic', statusecho_topic)
    assert_true(echo_topic.publish(msg))
    recv = echo_topic.get()
    print(recv)
    assert_true(recv.error == 'BAD_ERROR')
    assert_true(recv.code == 7)
    assert_true(recv.message == 'A bad Error happened')



if __name__ == '__main__':

    import nose
    nose.runmodule()
