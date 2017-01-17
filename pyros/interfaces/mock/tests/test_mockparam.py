from __future__ import absolute_import, print_function

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

import nose
from nose.tools import assert_true
import unittest

from pyros.interfaces.mock import MockSystem


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

    def test_echo_fortytwo(self):
        param = self.system.create_parameter('random_param', int)
        assert_true(param.setval(42))
        recv = param.getval()
        print(recv)
        assert_true(recv == 42)


if __name__ == '__main__':

    nose.runmodule()
