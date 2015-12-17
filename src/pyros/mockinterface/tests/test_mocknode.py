from __future__ import absolute_import
from __future__ import print_function

import sys
import os

import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'src')))

from pyros.mockinterface import PyrosMock
from pyros.mockinterface.mocktopic import statusecho_topic, MockTopic
import zmp
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

    mockn.start()
    try:
        assert_true(mockn.is_alive())

        print("Discovering msg_build Service...")
        msg_build = zmp.discover("msg_build", 5)  # we wait a bit to let it time to start
        assert_false(msg_build is None)
        assert_equal(len(msg_build.providers), 1)

        print("Discovering topic Service...")
        topic = zmp.discover("topic", 5)  # we wait a bit to let it time to start
        assert_false(topic is None)
        assert_equal(len(topic.providers), 1)

        print("Discovering topics Service...")
        topic_list = zmp.discover("topics", 5)  # we wait a bit to let it time to start
        assert_false(topic_list is None)
        assert_equal(len(topic_list.providers), 1)

        print("Discovering service Service...")
        service = zmp.discover("service", 5)  # we wait a bit to let it time to start
        assert_false(service is None)
        assert_equal(len(service.providers), 1)

        print("Discovering services Service...")
        service_list = zmp.discover("services", 5)  # we wait a bit to let it time to start
        assert_false(service_list is None)
        assert_equal(len(service_list.providers), 1)

        print("Discovering param Service...")
        param = zmp.discover("param", 5)  # we wait a bit to let it time to start
        assert_false(param is None)
        assert_equal(len(param.providers), 1)

        print("Discovering params Service...")
        param_list = zmp.discover("params", 5)  # we wait a bit to let it time to start
        assert_false(param_list is None)
        assert_equal(len(param_list.providers), 1)

        print("Discovering reinit Service...")
        param_list = zmp.discover("reinit", 5)  # we wait a bit to let it time to start
        assert_false(param_list is None)
        assert_equal(len(param_list.providers), 1)
    finally:
        mockn.shutdown()
        assert_false(mockn.is_alive())


def test_mocknode_topics_detect():  # Here we check that this node actually detects a topic
    mockn = PyrosMock()
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'topics'))
    # create a topic on that mock interface
    mockn.mock_if.mock_topic_appear('test_topic', statusecho_topic)
    mockn.reinit(services=[], topics=['test_topic'], params=[])
    mockn.start()
    try:
        assert_true(mockn.is_alive())

        print("Discovering topics Service...")
        topics = zmp.discover("topics", 5)  # we wait a bit to let it time to start
        assert_false(topics is None)
        assert_equal(len(topics.providers), 1)

        res = topics.call()
        assert_true('test_topic' in res)  # topic not detected since not in list of exposed topics


    finally:
        mockn.shutdown()
        assert_false(mockn.is_alive())


def test_mocknode_topics_detect_reinit():  # Here we check that this node actually detects a topic upon reinit
    mockn = PyrosMock()
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'topics'))

    mockn.start()
    try:
        assert_true(mockn.is_alive())

        print("Discovering mock_topic_appear Service...")
        mock_topic_appear = zmp.discover("mock_topic_appear", 5)  # we wait a bit to let it time to start
        assert_false(mock_topic_appear is None)
        assert_equal(len(mock_topic_appear.providers), 1)

        # create a topic on that mock process
        mock_topic_appear.call(args=('test_topic', statusecho_topic))

        print("Discovering topics Service...")
        topics = zmp.discover("topics", 5)  # we wait a bit to let it time to start
        assert_false(topics is None)
        assert_equal(len(topics.providers), 1)

        res = topics.call()
        assert_true(not 'test_topic' in res)  # topic not detected since not in list of exposed topics

        print("Discovering reinit Service...")
        reinit = zmp.discover("reinit", 5)  # we wait a bit to let it time to start
        assert_false(reinit is None)
        assert_equal(len(reinit.providers), 1)

        reinit.call(kwargs={'services': [], 'topics': ['test_topic'], 'params': []})

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

        print("Discovering mock_topic_appear Service...")
        mock_topic_appear = zmp.discover("mock_topic_appear", 3)  # we wait a bit to let it time to start
        assert_false(mock_topic_appear is None)
        assert_equal(len(mock_topic_appear.providers), 1)

        print("Discovering topics Service...")
        topics = zmp.discover("topics", 3)  # we wait a bit to let it time to start
        assert_false(topics is None)
        assert_equal(len(topics.providers), 1)

        print("Discovering reinit Service...")
        reinit = zmp.discover("reinit", 3)  # we wait a bit to let it time to start
        assert_false(reinit is None)
        assert_equal(len(reinit.providers), 1)

        reinit.call(kwargs={'services': [], 'topics': ['test_topic'], 'params': []})

        # create a topic on that mock process
        mock_topic_appear.call(args=('test_topic', statusecho_topic))

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

# TODO : test each service

if __name__ == '__main__':

    import nose
    nose.runmodule()




