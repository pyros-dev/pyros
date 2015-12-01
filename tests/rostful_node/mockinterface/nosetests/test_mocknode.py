from __future__ import absolute_import
from __future__ import print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import zmp
from rostful_node.mockinterface import PyrosMock
from nose.tools import timed, assert_true, assert_false, assert_equal, assert_raises


### TESTING NODE CREATION / TERMINATION ###
# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mocknode_creation_termination():
    mockn = PyrosMock()
    assert_false(mockn.is_alive())
    mockn.start()
    assert_true(mockn.is_alive())
    mockn.shutdown()
    assert_false(mockn.is_alive())

@timed(5)
def test_mocknode_provide_services(): # Here we check that this node actually provides all the services
    mockn = PyrosMock()
    assert_false(mockn.is_alive())

    assert_true(hasattr(mockn, 'msg_build'))
    assert_true(hasattr(mockn, 'topic'))
    assert_true(hasattr(mockn, 'topic_list'))
    assert_true(hasattr(mockn, 'service'))
    assert_true(hasattr(mockn, 'service_list'))
    assert_true(hasattr(mockn, 'param'))
    assert_true(hasattr(mockn, 'param_list'))

    mockn.start()
    assert_true(mockn.is_alive())

    print("Discovering msg_build Service...")
    msg_build = zmp.discover("msg_build", 5)  # we wait a bit to let it time to start
    assert_false(msg_build is None)
    assert_equal(len(msg_build.providers), 1)

    print("Discovering topic Service...")
    topic = zmp.discover("topic", 5)  # we wait a bit to let it time to start
    assert_false(topic is None)
    assert_equal(len(topic.providers), 1)

    print("Discovering topic_list Service...")
    topic_list = zmp.discover("topic_list", 5)  # we wait a bit to let it time to start
    assert_false(topic_list is None)
    assert_equal(len(topic_list.providers), 1)

    print("Discovering service Service...")
    service = zmp.discover("service", 5)  # we wait a bit to let it time to start
    assert_false(service is None)
    assert_equal(len(service.providers), 1)

    print("Discovering service_list Service...")
    service_list = zmp.discover("service_list", 5)  # we wait a bit to let it time to start
    assert_false(service_list is None)
    assert_equal(len(service_list.providers), 1)

    print("Discovering param Service...")
    param = zmp.discover("param", 5)  # we wait a bit to let it time to start
    assert_false(param is None)
    assert_equal(len(param.providers), 1)

    print("Discovering param_list Service...")
    param_list = zmp.discover("param_list", 5)  # we wait a bit to let it time to start
    assert_false(param_list is None)
    assert_equal(len(param_list.providers), 1)

    mockn.shutdown()
    assert_false(mockn.is_alive())


if __name__ == '__main__':

    import nose
    nose.runmodule()




