from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import zmp

from pyros.mockinterface import PyrosMock
from pyros.pyros_node import PyrosNode
from pyros.pyros_prtcl import MsgBuild, Topic, Service


def test_msg_build():
    msg = PyrosMock().msg_build('fake_connec_name')
    print "msg is of type {0}".format(type(msg))
    assert isinstance(msg, str)


def test_echo_topic_default():
    mock = PyrosMock()
    recv_msg = mock.topic('random_topic')
    assert recv_msg is None


def test_echo_same_topic():
    msg = 'testing'
    mock = PyrosMock()
    mock.topic('random_topic', msg)
    print "msg sent is {0}".format(msg)
    recv_msg = mock.topic('random_topic')
    print "msg received is {0}".format(recv_msg)
    assert msg == recv_msg


def test_other_topic():
    msg = 'testing'
    mock = PyrosMock()
    mock.topic('random_topic', msg)
    print "msg sent is {0}".format(msg)
    recv_msg = mock.topic('random_topic_2')
    print "msg received is {0}".format(recv_msg)
    assert recv_msg is None


def test_echo_service_default():
    msg = 'testing'
    mock = PyrosMock()
    assert mock.service('random_service') is None


def test_echo_service():
    msg = 'testing'
    mock = PyrosMock()
    print "msg sent is {0}".format(msg)
    recv_msg = mock.service('random_service', msg)
    print "msg received is {0}".format(recv_msg)
    assert msg == recv_msg


class TestRostfulMockProcess(object):
    def setUp(self):
        self.mockInstance = PyrosNode(mock=True)
        self.mockInstance.launch()

    def tearDown(self):
        self.mockInstance.shutdown()

    def test_msg_build(self):
        msg_build_svc = zmp.Service.discover('msg_build', self.mockInstance.getNodeName())
        resp = msg_build_svc.call(args=('fake_connec_name',))
        assert isinstance(resp, str)

    def test_echo_topic(self):
        topic_svc = zmp.Service.discover('topic', self.mockInstance.getNodeName())
        resp = topic_svc.call(args=('random_topic', 'testing'))
        assert resp is None  # message consumed

        resp = topic_svc.call(args=('random_topic', None))
        assert resp == 'testing'  # message echoed

    def test_other_topic(self):
        topic_svc = zmp.Service.discover('topic', self.mockInstance.getNodeName())
        resp = topic_svc.call(args=('random_topic', 'testing'))
        assert resp is None  # message consumed

        resp = topic_svc.call(args=('random_topic_2', None))
        assert resp is None  # message not echoed

    def test_echo_service(self):
        service_svc = zmp.Service.discover('service', self.mockInstance.getNodeName())
        resp = service_svc.call(args=('random_service', 'testing'))
        assert resp == 'testing'  # message echoed

if __name__ == '__main__':

    import nose
    nose.runmodule()
