#!/usr/bin/env python
from __future__ import absolute_import

import os
import sys
import logging


# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

# Unit test import (  will emulate ROS setup if needed )
import time
from pyros import PyrosROS

# we import pyros.rosinterface here only to get the ROs setup done if needed,
# because we need to use ros modules for testing here
import pyros.rosinterface

import rospy
import roslaunch
import rosnode
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

import pyzmp

from pyros.rosinterface.rostests import Timeout

# useful test tools
from pyros_setup import rostest_nose
import unittest
import nose

# Ref : http://pythontesting.net/framework/nose/nose-fixture-reference/


# Fixtures should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Note : rospy.init_node is forbidden here because it would prevent the PyrosROS Process to start()


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request


@nose.tools.nottest
class TestPyrosROS(object):
    """
    Main test fixture holding all tests
    Subclasses can override setup / teardown to test different environments
    """
    launch = None

    # Class fixture ( once each )
    @classmethod
    def setup_class(cls):
        # Start roslaunch
        TestPyrosROS.launch = roslaunch.scriptapi.ROSLaunch()
        TestPyrosROS.launch.start()

        # Note : rospy.init_node is forbidden here because it would prevent the PyrosROS Process to start()
        # Also we cannot use rospy.init_node in more than 1 test here

    @classmethod
    def teardown_class(cls):
        pass

    # Methods fixtures ( once per test method )
    def setUp(self, enable_cache=False):
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self.enable_cache = enable_cache

    def tearDown(self):
        pass

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    @nose.tools.timed(5)
    def test_rosnode_creation_termination(self):
        rosn = PyrosROS()
        nose.tools.assert_true(not rosn.is_alive())
        try:
            rosn.start()
            nose.tools.assert_true(rosn.is_alive())

        finally:
            # finishing PyrosROS process
            if rosn is not None and rosn.is_alive():
                rosn.shutdown()

        nose.tools.assert_true(not rosn.is_alive())

    @nose.tools.timed(5)
    def test_rosnode_provide_services(self):  # Here we check that this node actually provides all the services
        rosn = PyrosROS()
        nose.tools.assert_true(not rosn.is_alive())
        try:
            rosn.start()
            nose.tools.assert_true(rosn.is_alive())

            print("Discovering msg_build Service...")
            msg_build = pyzmp.discover("msg_build", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(msg_build is not None)
            print("msg_build providers : {svc}".format(svc=msg_build.providers))
            nose.tools.assert_equal(len(msg_build.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in msg_build.providers])

            print("Discovering topic Service...")
            topic = pyzmp.discover("topic", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(topic is not None)
            print("topic providers : {svc}".format(svc=topic.providers))
            nose.tools.assert_equal(len(topic.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in topic.providers])

            print("Discovering topics Service...")
            topics = pyzmp.discover("topics", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(topics is not None)
            print("topics providers : {svc}".format(svc=topics.providers))
            nose.tools.assert_equal(len(topics.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in topics.providers])

            print("Discovering service Service...")
            service = pyzmp.discover("service", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(service is not None)
            print("service providers : {svc}".format(svc=service.providers))
            nose.tools.assert_equal(len(service.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in service.providers])

            print("Discovering services Service...")
            services = pyzmp.discover("services", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(services is not None)
            print("services providers : {svc}".format(svc=services.providers))
            nose.tools.assert_equal(len(services.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in services.providers])

            print("Discovering param Service...")
            param = pyzmp.discover("param", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(param is not None)
            print("param providers : {svc}".format(svc=param.providers))
            nose.tools.assert_equal(len(param.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in param.providers])

            print("Discovering params Service...")
            params = pyzmp.discover("params", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(params is not None)
            print("params providers : {svc}".format(svc=params.providers))
            nose.tools.assert_equal(len(params.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in params.providers])

            print("Discovering setup Service...")
            setup = pyzmp.discover("setup", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(setup is not None)
            print("setup providers : {svc}".format(svc=setup.providers))
            nose.tools.assert_equal(len(setup.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in setup.providers])
        finally:
            # finishing PyrosROS process
            if rosn is not None and rosn.is_alive():
                rosn.shutdown()

        nose.tools.assert_true(not rosn.is_alive())

    def test_rosnode_topics(self):  # Here we check that this node actually discovers topics

        # Starting underlying system before
        rospy.set_param('/string_pub/topic_name', '~test_str_topic')  # private topic name to not mess things up too much
        rospy.set_param('/string_pub/test_message', 'testing topic discovery')
        string_pub_node = roslaunch.core.Node('pyros_test', 'string_pub_node.py', name='string_pub')
        string_pub_process = self.launch.launch(string_pub_node)
        try:
            # Starting PyrosROS with preconfigured topics,
            rosn = PyrosROS(kwargs={'topics': ['/string_pub/test_str_topic'], 'enable_cache': self.enable_cache})  # careful assuming the topic fullname here
            try:
                nose.tools.assert_true(not rosn.is_alive())
                rosn.start()
                nose.tools.assert_true(rosn.is_alive())

                print("Discovering topics Service...")
                topics = pyzmp.discover("topics", 5)  # we wait a bit to let it time to start
                nose.tools.assert_true(topics is not None)
                print("topics providers : {svc}".format(svc=topics.providers))
                nose.tools.assert_equal(len(topics.providers), 1)
                nose.tools.assert_true(rosn.name in [p[0] for p in topics.providers])

                res = topics.call()
                # What we get here is non deterministic
                # however we can wait for topic to be detected to make sure we get it after some time

                with Timeout(15) as t:
                    while not t.timed_out and not '/string_pub/test_str_topic' in res.keys():
                        rospy.rostime.wallsleep(1)
                        res = topics.call()

                nose.tools.assert_true('/string_pub/test_str_topic' in res.keys())  # test_topic has been created, detected and exposed
            finally:
                # finishing PyrosROS process
                if rosn is not None and rosn.is_alive():
                    rosn.shutdown()

            nose.tools.assert_true(not rosn.is_alive())

        finally:
            # finishing string_pub_process
            if string_pub_process is not None and string_pub_process.is_alive():
                string_pub_process.stop()

            nose.tools.assert_true(not string_pub_process.is_alive())

    def test_rosnode_topics_setup(self):  # Here we check that this node actually provides all the services
        rosn = PyrosROS()
        try:
            nose.tools.assert_true(not rosn.is_alive())
            rosn.start()
            nose.tools.assert_true(rosn.is_alive())

            print("Discovering topics Service...")
            topics = pyzmp.discover("topics", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(topics is not None)
            print("topics providers : {svc}".format(svc=topics.providers))
            nose.tools.assert_equal(len(topics.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in topics.providers])

            res = topics.call()
            nose.tools.assert_true('test_topic' not in res.keys())  # test_topic has not been created, detected or exposed

            print("Discovering setup Service...")
            setup = pyzmp.discover("setup", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(setup is not None)
            print("setup providers : {svc}".format(svc=setup.providers))
            nose.tools.assert_equal(len(setup.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in setup.providers])

            rospy.set_param('/string_pub/topic_name', '~test_str_topic')
            rospy.set_param('/string_pub/test_message', 'testing topic discovery')
            string_pub_node = roslaunch.core.Node('pyros_test', 'string_pub_node.py', name='string_pub')
            string_pub_process = self.launch.launch(string_pub_node)
            try:

                new_config = setup.call(kwargs={
                    'services': [],
                    'topics': ['/string_pub/test_str_topic'],
                    'params': [],
                    'enable_cache': self.enable_cache
                })
                # What we get here is non deterministic
                # however we can wait for topic to be detected to make sure we get it after some time

                res = topics.call()
                with Timeout(15) as t:
                    while not t.timed_out and not '/string_pub/test_str_topic' in res.keys():
                        rospy.rostime.wallsleep(1)
                        res = topics.call()

                nose.tools.assert_true('/string_pub/test_str_topic' in res.keys())  # test_topic has been created, detected and exposed

            finally:
                # finishing all processes
                if string_pub_process is not None and string_pub_process.is_alive():
                    string_pub_process.stop()

            nose.tools.assert_true(not string_pub_process.is_alive())
        finally:
            # finishing PyrosROS process
            if rosn is not None and rosn.is_alive():
                rosn.shutdown()

        nose.tools.assert_true(not rosn.is_alive())
        # TODO : do we need a test with subscriber ?

    def test_rosnode_services(self):  # Here we check that this node actually discovers topics

        # Starting underlying system before
        rospy.set_param('/string_echo/topic_name', '~topic')  # private names to not mess things up too much
        rospy.set_param('/string_echo/echo_topic_name', '~echo_topic')
        rospy.set_param('/string_echo/echo_service_name', '~echo_service')

        string_echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='string_echo')
        try:
            string_echo_process = self.launch.launch(string_echo_node)
        except roslaunch.RLException as rlexc:
            logging.error("pyros_test is needed to run this test. Please verify that it is installed in your ROS environment")
            raise
        try:
            # Starting PyrosROS with preconfigured services to expose
            rosn = PyrosROS(kwargs={'services': ['/string_echo/echo_service'], 'enable_cache': self.enable_cache})  # careful assuming the service fullname here
            try:
                nose.tools.assert_true(not rosn.is_alive())
                rosn.start()
                nose.tools.assert_true(rosn.is_alive())

                print("Discovering services Service...")
                services = pyzmp.discover("services", 5)  # we wait a bit to let it time to start
                nose.tools.assert_true(services is not None)
                print("services providers : {svc}".format(svc=services.providers))
                nose.tools.assert_equal(len(services.providers), 1)
                nose.tools.assert_true(rosn.name in [p[0] for p in services.providers])

                res = services.call()
                # What we get here is non deterministic
                # however we can wait for service to be detected to make sure we get it after some time

                with Timeout(5) as t:
                    while not t.timed_out and not '/string_echo/echo_service' in res.keys():
                        rospy.rostime.wallsleep(1)
                        res = services.call()

                nose.tools.assert_true('/string_echo/echo_service' in res.keys())  # echo_service has been created, detected and exposed
            finally:
                # finishing PyrosROS process
                if rosn is not None and rosn.is_alive():
                    rosn.shutdown()

            nose.tools.assert_true(not rosn.is_alive())

        finally:
            # finishing string_pub_process
            if string_echo_process is not None and string_echo_process.is_alive():
                string_echo_process.stop()

        nose.tools.assert_true(not string_echo_process.is_alive())

    def test_rosnode_services_setup(self):  # Here we check that this node actually provides all the services
        rosn = PyrosROS()
        try:
            nose.tools.assert_true(not rosn.is_alive())
            rosn.start()
            nose.tools.assert_true(rosn.is_alive())

            print("Discovering services Service...")
            services = pyzmp.discover("services", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(services is not None)
            print("services providers : {svc}".format(svc=services.providers))
            nose.tools.assert_equal(len(services.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in services.providers])

            res = services.call()
            nose.tools.assert_true('echo_service' not in res.keys())  # test_topic has not been created, detected or exposed

            print("Discovering setup Service...")
            setup = pyzmp.discover("setup", 5)  # we wait a bit to let it time to start
            nose.tools.assert_true(setup is not None)
            print("setup providers : {svc}".format(svc=setup.providers))
            nose.tools.assert_equal(len(setup.providers), 1)
            nose.tools.assert_true(rosn.name in [p[0] for p in setup.providers])

            rospy.set_param('/string_echo/topic_name', '~topic')  # private names to not mess things up too much
            rospy.set_param('/string_echo/echo_topic_name', '~echo_topic')
            rospy.set_param('/string_echo/echo_service_name', '~echo_service')

            string_echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='string_echo')
            string_echo_process = self.launch.launch(string_echo_node)
            try:

                new_config = setup.call(kwargs={
                    'services': ['/string_echo/echo_service'],
                    'topics': [],
                    'params': [],
                    'enable_cache': self.enable_cache,
                })
                # What we get here is non deterministic
                # however we can wait for topic to be detected to make sure we get it after some time

                res = services.call()

                with Timeout(5) as t:
                    while not t.timed_out and not '/string_echo/echo_service' in res.keys():
                        rospy.rostime.wallsleep(1)
                        res = services.call()

                nose.tools.assert_true('/string_echo/echo_service' in res.keys())  # test_topic has been created, detected and exposed

            finally:
                # finishing all processes
                if string_echo_process is not None and string_echo_process.is_alive():
                    string_echo_process.stop()

            nose.tools.assert_true(not string_echo_process.is_alive())

        finally:
            # finishing PyrosROS process
            if rosn is not None and rosn.is_alive():
                rosn.shutdown()

        nose.tools.assert_true(not rosn.is_alive())


# TODO : test each service

# TODO : test the update() is actually throttled (careful about cache behavior : no throttling)

# TODO : Test appearing / disappearing ROS topics / services

@nose.tools.istest
class TestPyrosROSNoCache(TestPyrosROS):

    # Methods fixtures ( once per test method )
    def setUp(self):
        super(TestPyrosROSNoCache, self).setUp()

    def tearDown(self):
        super(TestPyrosROSNoCache, self).tearDown()


if __name__ == '__main__':

    import nose
    nose.runmodule()




