from __future__ import absolute_import
from __future__ import print_function

import os
import sys

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

import time

from pyros.mockinterface.mocksystem import (
    mock_service_remote, mock_topic_remote, mock_param_remote,
    services_available_remote, services_available_type_remote,
    topics_available_remote, topics_available_type_remote,
    params_available_remote, params_available_type_remote,
)
from pyros import PyrosMock
from pyros.mockinterface.mocktopic import statusecho_topic, MockTopic
import pyzmp
from nose.tools import timed, assert_true, assert_false, assert_equal, assert_raises


### TESTING NODE CREATION / TERMINATION ###
# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mocknode_creation_termination():
    mockn = PyrosMock()
    assert_false(mockn.is_alive())
    mockn.start()
    try:
        assert_true(mockn.is_alive())
    finally:
        mockn.shutdown()
        assert_false(mockn.is_alive())


@timed(5)
def test_mocknode_provide_services():  # Here we check that this node actually provides all the services
    mockn = PyrosMock()
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'msg_build'))
    assert_true(hasattr(mockn, 'topic'))
    assert_true(hasattr(mockn, 'topics'))
    assert_true(hasattr(mockn, 'service'))
    assert_true(hasattr(mockn, 'services'))
    assert_true(hasattr(mockn, 'param'))
    assert_true(hasattr(mockn, 'params'))
    assert_true(hasattr(mockn, 'setup'))

    mockn.start()
    try:
        assert_true(mockn.is_alive())

        print("Discovering msg_build Service...")
        msg_build = pyzmp.discover("msg_build", 5)  # we wait a bit to let it time to start
        assert_false(msg_build is None)
        assert_equal(len(msg_build.providers), 1)

        print("Discovering topic Service...")
        topic = pyzmp.discover("topic", 5)  # we wait a bit to let it time to start
        assert_false(topic is None)
        assert_equal(len(topic.providers), 1)

        print("Discovering topics Service...")
        topic_list = pyzmp.discover("topics", 5)  # we wait a bit to let it time to start
        assert_false(topic_list is None)
        assert_equal(len(topic_list.providers), 1)

        print("Discovering service Service...")
        service = pyzmp.discover("service", 5)  # we wait a bit to let it time to start
        assert_false(service is None)
        assert_equal(len(service.providers), 1)

        print("Discovering services Service...")
        service_list = pyzmp.discover("services", 5)  # we wait a bit to let it time to start
        assert_false(service_list is None)
        assert_equal(len(service_list.providers), 1)

        print("Discovering param Service...")
        param = pyzmp.discover("param", 5)  # we wait a bit to let it time to start
        assert_false(param is None)
        assert_equal(len(param.providers), 1)

        print("Discovering params Service...")
        param_list = pyzmp.discover("params", 5)  # we wait a bit to let it time to start
        assert_false(param_list is None)
        assert_equal(len(param_list.providers), 1)

        print("Discovering setup Service...")
        param_list = pyzmp.discover("setup", 5)  # we wait a bit to let it time to start
        assert_false(param_list is None)
        assert_equal(len(param_list.providers), 1)
    finally:
        mockn.shutdown()
        assert_false(mockn.is_alive())


def test_mocknode_topics_detect():  # Here we check that this node actually detects a topic
    mockn = PyrosMock(kwargs={
        'services': [],
        'topics': ['test_topic'],
        'params': []
    })
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'topics'))

    # starting the node
    mockn.start()

    # checking interface is still None here ( instantiated in child only )
    assert_true(mockn.interface is None)

    # Services are initialized in run() method of pyzmp.Node, after interface has been initialized
    try:
        assert_true(mockn.is_alive())

        with mock_topic_remote('test_topic', statusecho_topic):

            # asserting the mock system has done its job from our point of view at least
            assert_true('test_topic' in topics_available_remote)
            assert_equal(topics_available_type_remote['test_topic'], statusecho_topic)

            # Getting topics list from child process
            print("Discovering topics Service...")
            topics = pyzmp.discover("topics", 3)  # we wait a bit to let it time to start
            assert_false(topics is None)
            assert_equal(len(topics.providers), 1)

            time.sleep(mockn.update_interval + 1)  # make sure we let update time to kick in

            res = topics.call(recv_timeout=6000000)
            # the mock system should have done its job from the other process perspective too
            # via multiprocess manager list
            assert_true('test_topic' in res)  # topic detected since in list of exposed topics

    finally:
        mockn.shutdown()
        assert_false(mockn.is_alive())


def test_mocknode_topics_detect_setup():  # Here we check that this node actually detects a topic upon setup
    mockn = PyrosMock()
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'topics'))

    mockn.start()
    try:
        assert_true(mockn.is_alive())

        with mock_topic_remote('test_topic', statusecho_topic):

            print("Discovering topics Service...")
            topics = pyzmp.discover("topics", 3)  # we wait a bit to let it time to start
            assert_false(topics is None)
            assert_equal(len(topics.providers), 1)

            res = topics.call()
            assert_true(not 'test_topic' in res)  # topic not detected since not in list of exposed topics

            print("Discovering setup Service...")
            setup = pyzmp.discover("setup", 3)  # we wait a bit to let it time to start
            assert_false(setup is None)
            assert_equal(len(setup.providers), 1)

            setup.call(kwargs={'services': [], 'topics': ['test_topic'], 'params': []})

            time.sleep(mockn.update_interval + 1)  # waiting for update to kick in

            res = topics.call()
            assert_true('test_topic' in res)
    finally:
        mockn.shutdown()
        assert_false(mockn.is_alive())


def test_mocknode_topics_detect_throttled():
    """
    Testing that the mocknode detection of topics is throttled properly
    :return:
    """
    mockn = PyrosMock()
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'topics'))

    mockn.update_interval = 5  # we wait 5 seconds between each update_throttled call
    mockn.start()  # one update will be triggered, and then nothing for the next 10 seconds
    try:
        assert_true(mockn.is_alive())

        print("Discovering setup Service...")
        setup = pyzmp.discover("setup", 3)  # we wait a bit to let it time to start
        assert_false(setup is None)
        assert_equal(len(setup.providers), 1)

        setup.call(kwargs={'services': [], 'topics': ['test_topic'], 'params': []})

        with mock_topic_remote('test_topic', statusecho_topic):

            print("Discovering topics Service...")
            topics = pyzmp.discover("topics", 3)  # we wait a bit to let it time to start
            assert_false(topics is None)
            assert_equal(len(topics.providers), 1)

            # topic is very likely not detected yet ( we didn't wait after creating and exposing it )
            res = topics.call()
            assert_true(not 'test_topic' in res)

            time.sleep(mockn.update_interval + 1)  # make sure we let update time to kick in

            # topic has to be detected now
            res = topics.call()
            assert_true('test_topic' in res)

    finally:  # to make sure we clean up on failure
        mockn.shutdown()
        assert_false(mockn.is_alive())


def test_msg_build():
    msg = PyrosMock().msg_build('fake_connec_name')
    print("msg is of type {0}".format(type(msg)))
    assert isinstance(msg, str)


def test_echo_topic_default():
    mock = PyrosMock()
    recv_msg = mock.topic('random_topic')
    assert recv_msg is None


def test_echo_same_topic():
    msg = 'testing'
    mock = PyrosMock()
    mock.topic('random_topic', msg)
    print("msg sent is {0}".format(msg))
    recv_msg = mock.topic('random_topic')
    print("msg received is {0}".format(recv_msg))
    assert msg == recv_msg


def test_other_topic():
    msg = 'testing'
    mock = PyrosMock()
    mock.topic('random_topic', msg)
    print("msg sent is {0}".format(msg))
    recv_msg = mock.topic('random_topic_2')
    print("msg received is {0}".format(recv_msg))
    assert recv_msg is None


def test_echo_service_default():
    msg = 'testing'
    mock = PyrosMock()
    assert mock.service('random_service') is None


def test_echo_service():
    msg = 'testing'
    mock = PyrosMock()
    print("msg sent is {0}".format(msg))
    recv_msg = mock.service('random_service', msg)
    print("msg received is {0}".format(recv_msg))
    assert msg == recv_msg


class TestPyrosMockProcess(object):
    def setUp(self):
        self.mockInstance = PyrosMock()
        self.mockInstance.start()

    def tearDown(self):
        self.mockInstance.shutdown()

    def test_msg_build(self):
        msg_build_svc = pyzmp.Service.discover('msg_build', 5)
        assert(msg_build_svc is not None)
        assert(self.mockInstance not in msg_build_svc.providers)
        resp = msg_build_svc.call(args=('fake_connec_name',))
        assert isinstance(resp, str)

    def test_list_topic(self):
        list_topic_svc = pyzmp.Service.discover('topics', 5)
        assert(list_topic_svc is not None)
        assert(self.mockInstance not in list_topic_svc.providers)
        resp = list_topic_svc.call()
        assert resp is not None

    def test_echo_topic(self):
        topic_svc = pyzmp.Service.discover('topic', 5)
        assert(topic_svc is not None)
        assert(self.mockInstance not in topic_svc.providers)
        resp = topic_svc.call(args=('random_topic', 'testing'))
        assert resp is None  # message consumed

        resp = topic_svc.call(args=('random_topic', None))
        assert resp == 'testing'  # message echoed

    def test_other_topic(self):
        topic_svc = pyzmp.Service.discover('topic', 5)
        assert(topic_svc is not None)
        assert(self.mockInstance not in topic_svc.providers)
        resp = topic_svc.call(args=('random_topic', 'testing'))
        assert resp is None  # message consumed

        resp = topic_svc.call(args=('random_topic_2', None))
        assert resp is None  # message not echoed

    def test_list_service(self):
        service_svc = pyzmp.Service.discover('services', 5)
        assert(service_svc is not None)
        assert(self.mockInstance not in service_svc.providers)
        resp = service_svc.call()
        assert resp is not None  # message echoed

    def test_echo_service(self):
        service_svc = pyzmp.Service.discover('service', 5)
        assert(service_svc is not None)
        assert(self.mockInstance not in service_svc.providers)
        resp = service_svc.call(args=('random_service', 'testing'))
        assert resp == 'testing'  # message echoed

    #TODO Check that if a service is called inappropriately, the exception is properly transferred back to the calling process and reraised.

if __name__ == '__main__':

    import nose
    nose.runmodule()
