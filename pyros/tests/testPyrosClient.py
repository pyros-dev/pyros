from __future__ import absolute_import

import sys
import os

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

from pyros.pyros_mock import PyrosMock
from pyros.pyros_client import PyrosClient

import nose
from nose.tools import assert_equal, assert_raises


class TestPyrosClientOnMock(object):
    def setUp(self):
        self.mockInstance = PyrosMock()
        # setting up mock instance
        cmd_conn = self.mockInstance.start()
        self.client = PyrosClient(cmd_conn)

    def tearDown(self):
        self.mockInstance.shutdown()

    ### TOPICS ###

    # TODO : test list features more !
    def test_list_all(self):
        t = self.client.topics()
        # Make sure we get all mock topics
        assert t is not None


    def test_inject_None(self):  # injecting None is meaningless and should return false
        assert self.client.topic_inject('random_topic', None)  # simply check that it injected (default Empty since we support kwargs)

    def test_inject_Empty(self):
        assert self.client.topic_inject('random_topic')  # simply check if injected

    #TODO : how to test strict backend with Mock ?
    #def test_inject_Wrong(self):
    #    with assert_raises(Exception) as expt:  # TODO : be more specific
    #        data = 42
    #        self.client.topic_inject('random_topic', data)  # simply check exception raised
    #    # assert_equal(expt, smthg...)

    def test_extract_None(self):
        assert self.client.topic_extract('random_topic') is None  # simply check if nothing extracted

    def test_inject_extract_echo_Empty(self):
        assert self.client.topic_inject('random_topic')  # default should be {}
        print "injected message content {0}".format({})
        recv = self.client.topic_extract('random_topic')
        print "extracted message {0}".format(recv)
        assert recv == {}

    def test_inject_extract_echo_Simple_Arg(self):
        data = 'data_string'
        assert self.client.topic_inject('random_topic', data)
        print "injected message content {0}".format(data)
        recv = self.client.topic_extract('random_topic')
        print "extracted message content {0}".format(recv)
        assert recv == data

    def test_inject_extract_echo_Complex_Arg(self):
        data = {'first': 'first_string', 'second': 'second_string'}
        assert self.client.topic_inject('random_topic', data)
        print "injected message content {0}".format(data)
        recv = self.client.topic_extract('random_topic')
        print "extracted message content {0}".format(recv)
        assert recv == data

    def test_inject_extract_echo_Simple_KWArgs(self):
        assert self.client.topic_inject('random_topic', data='data_string')
        print "injected message content {0}".format("data='data_string'")
        recv = self.client.topic_extract('random_topic')
        print "extracted message content {0}".format(recv)
        assert recv == {'data':'data_string'}

    def test_inject_extract_echo_Complex_KWArgs(self):
        assert self.client.topic_inject('random_topic', first='first_string', second='second_string')
        print "injected message content {0}".format("first='first_string', second='second_string'")
        recv = self.client.topic_extract('random_topic')
        print "extracted message content {0}".format(recv)
        assert recv == {'first': 'first_string', 'second': 'second_string'}

    ### SERVICES ###
    # TODO : think how to test strict backend with Mock ?
    #def test_call_Wrong(self):
    #    with assert_raises(Exception) as expt:  # TODO : be more specific
    #        data = 42
    #        print "request content {0}".format(data)
    #        resp = self.client.service_call('random_service', data)  # simply check exception raised
    #    # assert_equal(expt, smthg...)

    def test_call_echo_Simple_Arg(self):
        data = 'data_string'
        print "request content {0}".format(data)
        resp = self.client.service_call('random_service', data)
        print "response content {0}".format(resp)
        assert resp == data

    def test_call_echo_None(self):
        print "request content {0}".format({})
        resp = self.client.service_call('random_service', None)  # calling with None is interpreted as default (Empty) call (since we support kwargs)
        print "response content {0}".format(resp)
        assert resp == {}

    def test_call_echo_Empty(self):
        print "request content {0}".format({})
        resp = self.client.service_call('random_service')
        print "response content {0}".format(resp)
        assert resp == {}

    def test_call_echo_Complex_Arg(self):
        data = {'first': 'first_string', 'second': 'second_string'}
        print "request content {0}".format(data)
        resp = self.client.service_call('random_service', data)
        print "response content {0}".format(resp)
        assert resp == data

    def test_call_echo_Simple_KWArgs(self):
        print "request content {0}".format("data='data_string'")
        resp = self.client.service_call('random_service', data='data_string')
        print "response content {0}".format(resp)
        assert resp == {'data': 'data_string'}

    def test_call_echo_Complex_KWArgs(self):
        print "injected message content {0}".format("first='first_string', second='second_string'")
        resp = self.client.service_call('random_service', first='first_string', second='second_string')
        print "extracted message content {0}".format(resp)
        assert resp == {'first': 'first_string', 'second': 'second_string'}

    ### PARAMS ###
    #def test_set_None(self):  # injecting None is meaningless and should return false
    #    assert not self.client.param_set('random_param', None)  # simply check that it didnt set

    def test_set_Empty(self):
        assert self.client.param_set('random_param')  # simply check if set

    def test_get_None(self):
        assert self.client.param_get('random_param') is None  # simply check if nothing got

    def test_set_get_echo_Empty(self):
        assert self.client.param_set('random_param')  # default should be {}
        print "set value content {0}".format({})
        recv = self.client.param_get('random_param')
        print "got value content {0}".format(recv)
        assert recv == {}

    def test_set_get_echo_Simple_Arg(self):
        data = 'data_string'
        assert self.client.param_set('random_param', data)
        print "set value content {0}".format(data)
        recv = self.client.param_get('random_param')
        print "got value content {0}".format(recv)
        assert recv == data

    def test_set_get_echo_Complex_Arg(self):
        data = {'first': 'first_string', 'second': 'second_string'}
        assert self.client.param_set('random_param', data)
        print "set value content {0}".format(data)
        recv = self.client.param_get('random_param')
        print "got value content {0}".format(recv)
        assert recv == data

    def test_set_get_echo_Simple_KWArgs(self):
        assert self.client.param_set('random_param', data='data_string')
        print "set value content {0}".format("data='data_string'")
        recv = self.client.param_get('random_param')
        print "got value content {0}".format(recv)
        assert recv == {'data':'data_string'}

    def test_set_get_echo_Complex_KWArgs(self):
        assert self.client.param_set('random_param', first='first_string', second='second_string')
        print "set value content {0}".format("first='first_string', second='second_string'")
        recv = self.client.param_get('random_param')
        print "got value content {0}".format(recv)
        assert recv == {'first': 'first_string', 'second': 'second_string'}

# TODO test service that throw exception