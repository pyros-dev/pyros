from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from rostful_node.rostful_node_process import RostfulNodeProcess
from rostful_node.rostful_client import RostfulClient


class TestRostfulClientOnMock(object):
    def setUp(self):
        self.mockInstance = RostfulNodeProcess(mock=True)
        cmd_conn = self.mockInstance.launch()
        self.client = RostfulClient(cmd_conn)

    def tearDown(self):
        self.mockInstance.terminate()

    def test_inject_None(self):  # injecting None is meaningless and should return false
        assert not self.client.topic_inject('random_topic', None)  # simply check that it didnt inject

    def test_inject_Empty(self):
        assert self.client.topic_inject('random_topic')  # simply check if injected

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

    def test_call_echo_None(self):
        print "request content {0}".format({})
        resp = self.client.service_call('random_service', None)  # calling with None is invalid and should return None
        print "response content {0}".format(resp)
        assert resp is None

    def test_call_echo_Empty(self):
        print "request content {0}".format({})
        resp = self.client.service_call('random_service')
        print "response content {0}".format(resp)
        assert resp == {}

    def test_call_echo_Simple_Arg(self):
        data = 'data_string'
        print "request content {0}".format(data)
        resp = self.client.service_call('random_service', data)
        print "response content {0}".format(resp)
        assert resp == data

    def test_call_echo_Complex_Arg(self):
        data = {'first': 'first_string', 'second': 'second_string'}
        print "request content {0}".format(data)
        resp = self.client.service_call('random_service', data)
        print "response content {0}".format(resp)
        assert resp == data

    def test_call_echo_Simple_KWArgs(self):
        print "request content {0}".format("data='data_string'")
        resp = self.client.service_call('random_topic', data='data_string')
        print "response content {0}".format(resp)
        assert resp == {'data': 'data_string'}

    def test_call_echo_Complex_KWArgs(self):
        print "injected message content {0}".format("first='first_string', second='second_string'")
        resp = self.client.service_call('random_topic', first='first_string', second='second_string')
        print "extracted message content {0}".format(resp)
        assert resp == {'first': 'first_string', 'second': 'second_string'}


