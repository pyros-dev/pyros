#!/usr/bin/env python
from __future__ import absolute_import

import sys
import logging

# Unit test import (  will emulate ROS setup if needed )
import time
from pyros.rosinterface import PyrosROS

import rospy
import roslaunch
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

import zmp
import dynamic_reconfigure.client as dynamic_reconfigure_client

# useful test tools
from pyros_setup import rostest_nose
import unittest
from nose.tools import assert_true, assert_equal, timed

# test node process not setup by default (rostest dont need it here)
launch = None


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        global launch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # Note : rospy.init_node is forbidden here because it would prevent the PyrosROS Process to start()
        # Also we cannot use rospy.init_node in more than 1 test here


def teardown_module():
    if not rostest_nose.is_rostest_enabled():

        rospy.signal_shutdown('test complete')

        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request


class TestPyrosROS(unittest.TestCase):
    def setUp(self):
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

    def tearDown(self):
        self.interface = None

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    @timed(5)
    def test_rosnode_creation_termination(self):
        rosn = PyrosROS(dynamic_reconfigure=False)
        assert_true(not rosn.is_alive())
        rosn.start()
        assert_true(rosn.is_alive())
        rosn.shutdown()
        assert_true(not rosn.is_alive())

    @timed(5)
    def test_rosnode_provide_services(self):  # Here we check that this node actually provides all the services
        rosn = PyrosROS(dynamic_reconfigure=False)
        assert_true(not rosn.is_alive())
        rosn.start()
        assert_true(rosn.is_alive())

        print("Discovering msg_build Service...")
        msg_build = zmp.discover("msg_build", 5)  # we wait a bit to let it time to start
        assert_true(msg_build is not None)
        print("msg_build providers : {svc}".format(svc=msg_build.providers))
        assert_equal(len(msg_build.providers), 1)
        assert_true(rosn.name in [p[0] for p in msg_build.providers])

        print("Discovering topic Service...")
        topic = zmp.discover("topic", 5)  # we wait a bit to let it time to start
        assert_true(topic is not None)
        print("topic providers : {svc}".format(svc=topic.providers))
        assert_equal(len(topic.providers), 1)
        assert_true(rosn.name in [p[0] for p in topic.providers])

        print("Discovering topics Service...")
        topics = zmp.discover("topics", 5)  # we wait a bit to let it time to start
        assert_true(topics is not None)
        print("topics providers : {svc}".format(svc=topics.providers))
        assert_equal(len(topics.providers), 1)
        assert_true(rosn.name in [p[0] for p in topics.providers])

        print("Discovering service Service...")
        service = zmp.discover("service", 5)  # we wait a bit to let it time to start
        assert_true(service is not None)
        print("service providers : {svc}".format(svc=service.providers))
        assert_equal(len(service.providers), 1)
        assert_true(rosn.name in [p[0] for p in service.providers])

        print("Discovering services Service...")
        services = zmp.discover("services", 5)  # we wait a bit to let it time to start
        assert_true(services is not None)
        print("services providers : {svc}".format(svc=services.providers))
        assert_equal(len(services.providers), 1)
        assert_true(rosn.name in [p[0] for p in services.providers])

        print("Discovering param Service...")
        param = zmp.discover("param", 5)  # we wait a bit to let it time to start
        assert_true(param is not None)
        print("param providers : {svc}".format(svc=param.providers))
        assert_equal(len(param.providers), 1)
        assert_true(rosn.name in [p[0] for p in param.providers])

        print("Discovering params Service...")
        params = zmp.discover("params", 5)  # we wait a bit to let it time to start
        assert_true(params is not None)
        print("params providers : {svc}".format(svc=params.providers))
        assert_equal(len(params.providers), 1)
        assert_true(rosn.name in [p[0] for p in params.providers])

        print("Discovering reinit Service...")
        reinit = zmp.discover("reinit", 5)  # we wait a bit to let it time to start
        assert_true(reinit is not None)
        print("reinit providers : {svc}".format(svc=reinit.providers))
        assert_equal(len(reinit.providers), 1)
        assert_true(rosn.name in [p[0] for p in reinit.providers])

        rosn.shutdown()
        assert_true(not rosn.is_alive())

    def test_rosnode_topics(self):  # Here we check that this node actually discovers topics

        # Starting underlying system before
        rospy.set_param('/string_pub/topic_name', '~test_str_topic')  # private topic name to not mess things up too much
        rospy.set_param('/string_pub/test_message', 'testing topic discovery')
        string_pub_node = roslaunch.core.Node('pyros', 'string_pub_node.py', name='string_pub')
        string_pub_process = launch.launch(string_pub_node)
        try:
            # Starting PyrosROS with preconfigured topics,
            # disabling dynamic_reconf to avoid override asynchronously on start().
            rosn = PyrosROS(dynamic_reconfigure=False)
            try:
                rosn.ros_if.reinit(topics=['/string_pub/test_str_topic'])  # careful assuming the topic fullname here
                assert_true(not rosn.is_alive())
                rosn.start()
                assert_true(rosn.is_alive())

                print("Discovering topics Service...")
                topics = zmp.discover("topics", 5)  # we wait a bit to let it time to start
                assert_true(topics is not None)
                print("topics providers : {svc}".format(svc=topics.providers))
                assert_equal(len(topics.providers), 1)
                assert_true(rosn.name in [p[0] for p in topics.providers])

                start = time.time()
                timeout = 15  # should be enough to let the node start (?)
                res = topics.call()
                # What we get here is non deterministic
                # however we can wait for topic to be detected to make sure we get it after some time

                while time.time() - start < timeout and not '/string_pub/test_str_topic' in res.keys():
                    rospy.rostime.wallsleep(1)
                    res = topics.call()

                assert_true('/string_pub/test_str_topic' in res.keys())  # test_topic has been created, detected and exposed
            finally:
                # finishing PyrosROS process
                if rosn is not None and rosn.is_alive():
                    rosn.shutdown()

        finally:
            # finishing string_pub_process
            if string_pub_process is not None and string_pub_process.is_alive():
                string_pub_process.stop()

    def test_rosnode_topics_reinit(self):  # Here we check that this node actually provides all the services
        rosn = PyrosROS(dynamic_reconfigure=False)
        try:
            assert_true(not rosn.is_alive())
            rosn.start()
            assert_true(rosn.is_alive())

            print("Discovering topics Service...")
            topics = zmp.discover("topics", 5)  # we wait a bit to let it time to start
            assert_true(topics is not None)
            print("topics providers : {svc}".format(svc=topics.providers))
            assert_equal(len(topics.providers), 1)
            assert_true(rosn.name in [p[0] for p in topics.providers])

            res = topics.call()
            assert_true('test_topic' not in res.keys())  # test_topic has not been created, detected or exposed

            print("Discovering reinit Service...")
            reinit = zmp.discover("reinit", 5)  # we wait a bit to let it time to start
            assert_true(reinit is not None)
            print("reinit providers : {svc}".format(svc=reinit.providers))
            assert_equal(len(reinit.providers), 1)
            assert_true(rosn.name in [p[0] for p in reinit.providers])

            rospy.set_param('/string_pub/topic_name', '~test_str_topic')
            rospy.set_param('/string_pub/test_message', 'testing topic discovery')
            string_pub_node = roslaunch.core.Node('pyros', 'string_pub_node.py', name='string_pub')
            string_pub_process = launch.launch(string_pub_node)
            try:

                new_config = reinit.call(kwargs={
                    'services': [],
                    'topics': ['/string_pub/test_str_topic'],
                    'params': []
                })
                # What we get here is non deterministic
                # however we can wait for topic to be detected to make sure we get it after some time

                start = time.time()
                timeout = 15  # should be enough to let the node start (?)
                res = topics.call()
                while time.time() - start < timeout and not '/string_pub/test_str_topic' in res.keys():
                    rospy.rostime.wallsleep(1)
                    res = topics.call()

                assert_true('/string_pub/test_str_topic' in res.keys())  # test_topic has been created, detected and exposed

            finally:
                # finishing all processes
                if string_pub_process is not None and string_pub_process.is_alive():
                    string_pub_process.stop()
        finally:
            # finishing PyrosROS process
            if rosn is not None and rosn.is_alive():
                rosn.shutdown()
        # TODO : do we need a test with subscriber ?

    def test_rosnode_services(self):  # Here we check that this node actually discovers topics

        # Starting underlying system before
        rospy.set_param('/string_echo/topic_name', '~topic')  # private names to not mess things up too much
        rospy.set_param('/string_echo/echo_topic_name', '~echo_topic')
        rospy.set_param('/string_echo/echo_service_name', '~echo_service')

        string_echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='string_echo')
        string_echo_process = launch.launch(string_echo_node)
        try:
            # Starting PyrosROS with preconfigured services,
            # disabling dynamic_reconf to avoid override asynchronously on start().
            rosn = PyrosROS(dynamic_reconfigure=False)
            try:
                rosn.ros_if.reinit(services=['/string_echo/echo_service'])  # careful assuming the service fullname here
                assert_true(not rosn.is_alive())
                rosn.start()
                assert_true(rosn.is_alive())

                print("Discovering services Service...")
                services = zmp.discover("services", 5)  # we wait a bit to let it time to start
                assert_true(services is not None)
                print("services providers : {svc}".format(svc=services.providers))
                assert_equal(len(services.providers), 1)
                assert_true(rosn.name in [p[0] for p in services.providers])

                start = time.time()
                timeout = 15  # should be enough to let the node start (?)
                res = services.call()
                # What we get here is non deterministic
                # however we can wait for service to be detected to make sure we get it after some time

                while time.time() - start < timeout and not '/string_echo/echo_service' in res.keys():
                    rospy.rostime.wallsleep(1)
                    res = services.call()

                assert_true('/string_echo/echo_service' in res.keys())  # echo_service has been created, detected and exposed
            finally:
                # finishing PyrosROS process
                if rosn is not None and rosn.is_alive():
                    rosn.shutdown()

        finally:
            # finishing string_pub_process
            if string_echo_process is not None and string_echo_process.is_alive():
                string_echo_process.stop()

    def test_rosnode_services_reinit(self):  # Here we check that this node actually provides all the services
        rosn = PyrosROS(dynamic_reconfigure=False)
        try:
            assert_true(not rosn.is_alive())
            rosn.start()
            assert_true(rosn.is_alive())

            print("Discovering services Service...")
            services = zmp.discover("services", 5)  # we wait a bit to let it time to start
            assert_true(services is not None)
            print("services providers : {svc}".format(svc=services.providers))
            assert_equal(len(services.providers), 1)
            assert_true(rosn.name in [p[0] for p in services.providers])

            res = services.call()
            assert_true('echo_service' not in res.keys())  # test_topic has not been created, detected or exposed

            print("Discovering reinit Service...")
            reinit = zmp.discover("reinit", 5)  # we wait a bit to let it time to start
            assert_true(reinit is not None)
            print("reinit providers : {svc}".format(svc=reinit.providers))
            assert_equal(len(reinit.providers), 1)
            assert_true(rosn.name in [p[0] for p in reinit.providers])

            rospy.set_param('/string_echo/topic_name', '~topic')  # private names to not mess things up too much
            rospy.set_param('/string_echo/echo_topic_name', '~echo_topic')
            rospy.set_param('/string_echo/echo_service_name', '~echo_service')

            string_echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='string_echo')
            string_echo_process = launch.launch(string_echo_node)
            try:

                new_config = reinit.call(kwargs={
                    'services': ['/string_echo/echo_service'],
                    'topics': [],
                    'params': []
                })
                # What we get here is non deterministic
                # however we can wait for topic to be detected to make sure we get it after some time

                start = time.time()
                timeout = 15  # should be enough to let the node start (?)
                res = services.call()
                while time.time() - start < timeout and not '/string_echo/echo_service' in res.keys():
                    rospy.rostime.wallsleep(1)
                    res = services.call()

                assert_true('/string_echo/echo_service' in res.keys())  # test_topic has been created, detected and exposed

            finally:
                # finishing all processes
                if string_echo_process is not None and string_echo_process.is_alive():
                    string_echo_process.stop()
        finally:
            # finishing PyrosROS process
            if rosn is not None and rosn.is_alive():
                rosn.shutdown()


# TODO : test each service

# TODO : test the update() is actually throttled (careful about cache behavior : no throttling)



# Testing with Connection Cache
class TestPyrosROSCache(TestPyrosROS):
    def setUp(self):
        self.connection_cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py', name='connection_cache',
                                                         remap_args=[('/rocon/connection_cache/list', '/pyros_ros/connections_list'),
                                                                     ('/rocon/connection_cache/diff', '/pyros_ros/connections_diff'),
                                                                     ])
        self.connection_cache_proc = launch.launch(self.connection_cache_node)

        super(TestPyrosROSCache, self).setUp()

    def tearDown(self):
        super(TestPyrosROSCache, self).tearDown()

        self.connection_cache_proc.stop()


if __name__ == '__main__':

    import nose
    nose.runmodule()




