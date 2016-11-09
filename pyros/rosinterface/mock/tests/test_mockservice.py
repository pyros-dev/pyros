from __future__ import absolute_import, print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

import nose
from nose.tools import assert_true, assert_false, assert_raises
import unittest

from pyros.rosinterface.mock import extract_values, populate_instance, FieldTypeMismatchException, NonexistentFieldException, StatusMsg
from pyros.rosinterface.mock import statusecho_service, MockSystem


class TestMockService(unittest.TestCase):
    """
    Main test fixture holding all tests
    Subclasses can override setup / teardown to test different environments
    """

    # Class fixture ( once each )
    @classmethod
    def setup_class(cls):
        cls.system = MockSystem()

    @classmethod
    def teardown_class(cls):
        pass

    def test_echo_status(self):
        msg = populate_instance({
            'error': 'BAD_ERROR',
            'code': 7,
            'message': 'A bad Error happened'
        }, StatusMsg('ERROR', 42, 'details'))
        assert_true(isinstance(msg, StatusMsg))
        echo_service = self.system.create_service('random_service', statusecho_service)
        resp = echo_service.call(msg)
        print(resp)
        assert_true(resp.error == 'BAD_ERROR')
        assert_true(resp.code == 7)
        assert_true(resp.message == 'A bad Error happened')

# TODO : make sure we handle errors when the service is exposed, but not existing any longer

if __name__ == '__main__':

    nose.runmodule()

