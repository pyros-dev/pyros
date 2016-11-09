from __future__ import absolute_import, print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

import nose
import unittest
from nose.tools import assert_true, assert_false, assert_raises


from pyros.rosinterface.mock import extract_values, populate_instance, FieldTypeMismatchException, NonexistentFieldException, StatusMsg
from pyros.rosinterface.mock import statusecho_topic, MockSystem


class TestMockTopic(unittest.TestCase):
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
        echo_topic_pub = self.system.create_publisher('random_topic', statusecho_topic)
        echo_topic_sub = self.system.create_subscriber('random_topic', statusecho_topic)
        assert_true(echo_topic_pub.publish(msg))
        recv = echo_topic_sub.get()
        print(recv)
        assert_true(recv.error == 'BAD_ERROR')
        assert_true(recv.code == 7)
        assert_true(recv.message == 'A bad Error happened')


if __name__ == '__main__':

    nose.runmodule()
