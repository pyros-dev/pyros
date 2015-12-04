from __future__ import absolute_import, print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from pyros.mockinterface import extract_values, populate_instance, FieldTypeMismatchException, NonexistentFieldException, StatusMsg
from pyros.mockinterface import MockParam
from nose.tools import assert_true, assert_false, assert_raises


def test_echo_fortytwo():
    param = MockParam('random_param')
    assert_true(param.set(42))
    recv = param.get()
    print(recv)
    assert_true(recv == 42)



if __name__ == '__main__':

    import nose
    nose.runmodule()
