from __future__ import absolute_import, print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from pyros.mockinterface import extract_values, populate_instance, FieldTypeMismatchException, NonexistentFieldException, StatusMsg
from pyros.mockinterface import statusecho_service, MockService
from nose.tools import assert_true, assert_false, assert_raises


def test_echo_status():
    msg = populate_instance({
        'error': 'BAD_ERROR',
        'code': 7,
        'message': 'A bad Error happened'
    }, StatusMsg('ERROR', 42, 'details'))
    assert_true(isinstance(msg, StatusMsg))
    echo_service = MockService('random_service', statusecho_service)
    resp = echo_service.call(msg)
    print(resp)
    assert_true(resp.error == 'BAD_ERROR')
    assert_true(resp.code == 7)
    assert_true(resp.message == 'A bad Error happened')

# TODO : make sure we handle errors when the service is exposed, but not existing any longer

if __name__ == '__main__':

    import nose
    nose.runmodule()

