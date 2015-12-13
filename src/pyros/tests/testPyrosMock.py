from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import zmp

from pyros.mockinterface import PyrosMock


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
        self.mockInstance = PyrosMock()
        self.mockInstance.start()

    def tearDown(self):
        self.mockInstance.shutdown()

    def test_msg_build(self):
        msg_build_svc = zmp.Service.discover('msg_build', 5)
        assert(msg_build_svc is not None and self.mockInstance not in msg_build_svc.providers)
        resp = msg_build_svc.call(args=('fake_connec_name',))
        assert isinstance(resp, str)

    def test_list_topic(self):
        list_topic_svc = zmp.Service.discover('topics', 5)
        assert(list_topic_svc is not None and self.mockInstance not in list_topic_svc.providers)
        resp = list_topic_svc.call()
        assert resp is not None

    def test_echo_topic(self):
        topic_svc = zmp.Service.discover('topic', 5)
        assert(topic_svc is not None and self.mockInstance not in topic_svc.providers)
        resp = topic_svc.call(args=('random_topic', 'testing'))
        assert resp is None  # message consumed

        resp = topic_svc.call(args=('random_topic', None))
        assert resp == 'testing'  # message echoed

    def test_other_topic(self):
        topic_svc = zmp.Service.discover('topic', 5)
        assert(topic_svc is not None and self.mockInstance not in topic_svc.providers)
        resp = topic_svc.call(args=('random_topic', 'testing'))
        assert resp is None  # message consumed

        resp = topic_svc.call(args=('random_topic_2', None))
        assert resp is None  # message not echoed

    def test_list_service(self):
        service_svc = zmp.Service.discover('services', 5)
        assert(service_svc is not None and self.mockInstance not in service_svc.providers)
        resp = service_svc.call()
        assert resp is not None  # message echoed

    def test_echo_service(self):
        service_svc = zmp.Service.discover('service', 5)
        assert(service_svc is not None and self.mockInstance not in service_svc.providers)
        resp = service_svc.call(args=('random_service', 'testing'))
        assert resp == 'testing'  # message echoed

    #TODO Check that if a service is called inappropriately, the exception is properly transferred back to the calling process and reraised.

if __name__ == '__main__':

    import nose
    nose.runmodule()
