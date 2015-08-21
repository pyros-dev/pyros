from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from rostful_node.rostful_mock import RostfulMock
from rostful_node.rostful_node_process import RostfulNodeProcess
from rostful_node.rostful_prtcl import MsgBuild, Topic, Service

def test_msg_build():
    msg = RostfulMock().msg_build('fake_connec_name')
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, str)

def test_echo_topic_default():
    mock = RostfulMock()
    recv_msg = mock.topic('random_topic')
    assert recv_msg is None

def test_echo_same_topic():
    msg = 'testing'
    mock = RostfulMock()
    mock.topic('random_topic', msg)
    print "msg sent is {0}".format(msg)
    recv_msg = mock.topic('random_topic')
    print "msg received is {0}".format(recv_msg)
    assert msg == recv_msg

def test_other_topic():
    msg = 'testing'
    mock = RostfulMock()
    mock.topic('random_topic', msg)
    print "msg sent is {0}".format(msg)
    recv_msg = mock.topic('random_topic_2')
    print "msg received is {0}".format(recv_msg)
    assert recv_msg is None

def test_echo_service_default():
    msg = 'testing'
    mock = RostfulMock()
    assert mock.service('random_service') is None

def test_echo_service():
    msg = 'testing'
    mock = RostfulMock()
    print "msg sent is {0}".format(msg)
    recv_msg = mock.service('random_service', msg)
    print "msg received is {0}".format(recv_msg)
    assert msg == recv_msg

class TestRostfulMockProcess(object):
    def setUp(self):
        self.mockInstance = RostfulNodeProcess(mock=True)
        self.cmd_conn = self.mockInstance.launch()

    def tearDown(self):
        self.mockInstance.terminate()

    def test_msg_build(self):
        msg = MsgBuild(name='fake_connec_name', msg_content=None)
        print "msg sent is {0}".format(msg)
        self.cmd_conn.send(msg)
        recv_msg = self.cmd_conn.recv()
        print "msg received is {0}".format(recv_msg)
        assert recv_msg.name == msg.name and isinstance(recv_msg.msg_content, str)

    def test_echo_topic(self):
        msg = Topic(name='random_topic', msg_content='testing')
        print "msg sent is {0}".format(msg)
        self.cmd_conn.send(msg)
        recv_msg = self.cmd_conn.recv()
        print "msg sent is {0}".format(msg)
        assert recv_msg.name == msg.name and recv_msg.msg_content is None  # message consumed

        next_msg = Topic(name='random_topic', msg_content=None)
        print "next_msg sent is {0}".format(next_msg)
        self.cmd_conn.send(next_msg)
        recv_msg = self.cmd_conn.recv()
        print "msg received is {0}".format(recv_msg)
        assert recv_msg.name == next_msg.name and recv_msg.name == msg.name and msg.msg_content == recv_msg.msg_content  # message echoed

    def test_other_topic(self):
        msg = Topic(name='random_topic', msg_content='testing')
        print "msg sent is {0}".format(msg)
        self.cmd_conn.send(msg)
        recv_msg = self.cmd_conn.recv()
        print "msg sent is {0}".format(msg)
        assert recv_msg.name == msg.name and recv_msg.msg_content is None  # message consumed

        next_msg = Topic(name='random_topic_2', msg_content=None)
        print "next_msg sent is {0}".format(next_msg)
        self.cmd_conn.send(next_msg)
        recv_msg = self.cmd_conn.recv()
        print "msg received is {0}".format(recv_msg)
        assert recv_msg.name == next_msg.name and recv_msg.name != msg.name and recv_msg.msg_content is None  # message not echoed

    def test_echo_service(self):
        msg = Service(name='random_service', rqst_content='testing', resp_content=None)
        print "msg sent is {0}".format(msg)
        self.cmd_conn.send(msg)
        recv_msg = self.cmd_conn.recv()
        print "msg received is {0}".format(recv_msg)
        assert msg.name == recv_msg.name and msg.rqst_content == recv_msg.rqst_content and msg.rqst_content == recv_msg.resp_content

if __name__ == '__main__':

    import nose
    nose.runmodule()
