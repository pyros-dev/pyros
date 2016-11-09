#!/usr/bin/env python
from __future__ import absolute_import

import os
import sys
import pickle

# This is needed if running this test directly (without using nose loader)
# prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
# And we need our current module to be found first.
import time

# current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
# # if not current_path in sys.path:
# sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec

# setting up logging from this test
import logging.config
# Setting up logging for this test
# TODO : solve multiprocess problems... this seems not enough to get on console...
logging.config.dictConfig(
    {
        'version': 1,
        'formatters': {
            'verbose': {
                'format': '%(levelname)s %(asctime)s %(module)s %(process)d %(thread)d %(message)s'
            },
            'simple': {
                'format': '%(levelname)s %(name)s:%(message)s'
            },
        },
        'handlers': {
            'null': {
                'level': 'DEBUG',
                'class': 'logging.NullHandler',
            },
            'console': {
                'level': 'DEBUG',
                'class': 'logging.StreamHandler',
                'formatter': 'simple'
            },
        },
        'loggers': {
            'pyros_config': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            'pyros': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            'pyros.rosinterface': {
                'handlers': ['console'],
                'level': 'DEBUG',
            },
            'pyros.ros_interface': {
                'handlers': ['console'],
                'level': 'DEBUG',
            }
        }
    }
)


# Unit test import (  will emulate ROS setup if needed )
import nose

from pyros.baseinterface import DiffTuple
from pyros.rosinterface import RosServiceIfPool
from pyros.rosinterface.connection_cache_utils import connection_cache_proxy_create, connection_cache_marshall


import rospy
import roslaunch
import rosnode
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

from pyros.rosinterface.rostests import Timeout


# useful test tools
from pyros_utils import rostest_nose
import unittest


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

    # Providing services require us to be a node
    rospy.init_node('test_service_if_pool', argv=None, disable_signals=True)
    # CAREFUL : rospy.init_node should be done only once per PROCESS
    # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request


@nose.tools.nottest
class TestRosServiceIfPool(unittest.TestCase):
    """
    Main test fixture holding all tests
    Subclasses can override setup / teardown to test different environments
    """

    launch = None
    # test node process not setup by default (rostest dont need it here)
    empty_srv_process = None
    trigger_srv_process = None

    # Class fixture ( once each )
    @classmethod
    def setup_class(cls):
        # Start roslaunch
        TestRosServiceIfPool.launch = roslaunch.scriptapi.ROSLaunch()
        TestRosServiceIfPool.launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match
        global empty_srv_process, trigger_srv_process
        empty_srv_node = roslaunch.core.Node('pyros_test', 'emptyService.py', name='empty_service')
        trigger_srv_node = roslaunch.core.Node('pyros_test', 'triggerService.py', name='trigger_service')
        TestRosServiceIfPool.empty_srv_process = TestRosServiceIfPool.launch.launch(empty_srv_node)
        TestRosServiceIfPool.trigger_srv_process = TestRosServiceIfPool.launch.launch(trigger_srv_node)

    @classmethod
    def teardown_class(cls):
        # ensuring all process are finished
        if TestRosServiceIfPool.empty_srv_process is not None:
            TestRosServiceIfPool.empty_srv_process.stop()
        if TestRosServiceIfPool.trigger_srv_process is not None:
            TestRosServiceIfPool.trigger_srv_process.stop()

    # EXPOSE SERVICES + UPDATE Interface

    def test_service_appear_expose_update(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """

        servicename = '/test/empsrv'
        dt = self.service_if_pool.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # service backend has not been created since the update didn't run yet
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        # NOTE : We need to wait to make sure the tests nodes are started...
        with Timeout(5) as t:
            while not t.timed_out and servicename not in dt.added:
                services, service_types = self.get_system_state()
                dt = self.service_if_pool.update(services, service_types)
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # every exposed service should remain in the list of args ( in case regex match another service )
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # make sure the service backend has been created
        self.assertTrue(servicename in self.service_if_pool.services.keys())

        # cleaning up
        self.service_if_pool.expose_services([])
        # service should not be in the list of args any longer
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # service backend has been removed
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

    def test_service_appear_update_expose(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """
        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())
        # First update should not change state
        services, service_types = self.get_system_state()
        dt = self.service_if_pool.update(services, service_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        # create the service and then try exposing the service again, simulating
        # it coming online before expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            with Timeout(5) as t:
                while not t.timed_out and nonexistent_srv.resolved_name not in self.service_if_pool.services_available:
                    services, service_types = self.get_system_state()
                    dt = self.service_if_pool.update(services, service_types)
                    self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            # every added service should be in the list of args
            self.assertTrue(servicename not in self.service_if_pool.services_args)
            # the backend should not have been created
            self.assertTrue(servicename not in self.service_if_pool.services.keys())

            # here we are sure the interface knows the service is available
            # it will be exposed right now
            dt = self.service_if_pool.expose_services([servicename])
            self.assertTrue(servicename in dt.added)  # servicename added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every exposed service should remain in the list of args ( in case regex match another service )
            self.assertTrue(servicename in self.service_if_pool.services_args)
            # make sure the service backend has been created
            self.assertTrue(servicename in self.service_if_pool.services.keys())
        finally:
            nonexistent_srv.shutdown('testing complete')

    def test_service_expose_appear_update(self):
        """
        Test basic service adding functionality for a service which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())
        # First update should not change state
        services, service_types = self.get_system_state()
        dt = self.service_if_pool.update(services, service_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        self.service_if_pool.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())
        services, service_types = self.get_system_state()
        dt = self.service_if_pool.update(services, service_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # make sure the service is STILL in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # make sure the service backend has STILL not been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and nonexistent_srv.resolved_name not in dt.added:
                    services, service_types = self.get_system_state()
                    dt = self.service_if_pool.update(services, service_types)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent_srv added
            # every exposed service should remain in the list of args ( in case regex match another service )
            self.assertTrue(servicename in self.service_if_pool.services_args)
            # make sure the service backend has been created
            self.assertTrue(servicename in self.service_if_pool.services.keys())
        finally:
            nonexistent_srv.shutdown('testing complete')

    def test_service_withhold_update_disappear(self):
        """
        Test service witholding functionality for a service which doesnt exists anymore in
        the ros environment. Normal usecase.
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        self.service_if_pool.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # service backend has NOT been created yet
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        dt = DiffTuple([], [])
        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            # wait here until service actually appear in cache proxy
            with Timeout(5) as t:
                while not t.timed_out and nonexistent_srv.resolved_name not in dt.added:
                    services, service_types = self.get_system_state()
                    dt = self.service_if_pool.update(services, service_types)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent_srv added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every withhold service should STILL be in the list of args
            self.assertTrue(servicename in self.service_if_pool.services_args)
            # service backend has been created
            self.assertTrue(servicename in self.service_if_pool.services.keys())

            dt = self.service_if_pool.expose_services([])
            self.assertEqual(dt.added, [])  # nothing added
            self.assertTrue(nonexistent_srv.resolved_name in dt.removed)  # nonexistent_srv removed
            # every withhold service should NOT be in the list of args
            self.assertTrue(servicename not in self.service_if_pool.services_args)
            # service backend should be GONE
            self.assertTrue(servicename not in self.service_if_pool.services.keys())

            services, service_types = self.get_system_state()
            dt = self.service_if_pool.update(services, service_types)
            self.assertEqual(dt.added, [])  # nothing added
            self.assertEqual(dt.removed, [])  # nothing removed
            # every withhold service should STILL NOT be in the list of args
            self.assertTrue(servicename not in self.service_if_pool.services_args)
            # service backend should be GONE
            self.assertTrue(servicename not in self.service_if_pool.services.keys())
        finally:
            nonexistent_srv.shutdown('testing disappearing service')

        services, service_types = self.get_system_state()
        dt = self.service_if_pool.update(services, service_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nonexistent_srv already removed
        # every withhold service should STILL NOT be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # service backend should be GONE
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

    def test_service_disappear_update_withhold(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """
        servicename = '/test/absentsrv1'
        # service should not be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        self.service_if_pool.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # service backend has NOT been created yet
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        dt = DiffTuple([], [])
        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            # wait here until service actually appear in cache proxy
            with Timeout(5) as t:
                while not t.timed_out and nonexistent_srv.resolved_name not in dt.added:
                    services, service_types = self.get_system_state()
                    dt = self.service_if_pool.update(services, service_types)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent added
            self.assertEqual(dt.removed, [])  # nothing removed

            # service should be in the list of args
            self.assertTrue(servicename in self.service_if_pool.services_args)
            # the backend should have been created
            self.assertTrue(servicename in self.service_if_pool.services.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            nonexistent_srv.shutdown('testing disappearing service')

        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # the backend should STILL be there
        self.assertTrue(servicename in self.service_if_pool.services.keys())
        # Note the service implementation should take care of possible errors in this case

        # wait here until service actually disappear from cache proxy
        with Timeout(5) as t:
            while not t.timed_out and nonexistent_srv.resolved_name not in dt.removed:
                services, service_types = self.get_system_state()
                dt = self.service_if_pool.update(services, service_types)
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(nonexistent_srv.resolved_name in dt.removed)  # nonexistent_srv removed
        # every exposed service should remain in the list of args ( in case regex match another service )
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # make sure the service backend should NOT be there any longer
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        # TODO : test that coming back actually works

        self.service_if_pool.expose_services([])
        # every withhold service should NOT be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # service backend has not been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        services, service_types = self.get_system_state()
        dt = self.service_if_pool.update(services, service_types)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold service should NOT be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # make sure the service backend has been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

    def test_service_update_disappear_withhold(self):
        """
        Test service exposing functionality for a service which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """

        servicename = '/test/absentsrv1'
        # every added service should be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # the backend should not have been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        self.service_if_pool.expose_services([servicename])
        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # service backend has not been created
        self.assertTrue(servicename not in self.service_if_pool.services.keys())

        dt = DiffTuple([], [])
        # create the service and then try updating again, simulating
        # it coming online after expose call.
        nonexistent_srv = rospy.Service(servicename, EmptySrv, srv_cb)
        try:
            # wait here until service actually appear in cache proxy
            with Timeout(5) as t:
                while not t.timed_out and nonexistent_srv.resolved_name not in dt.added:
                    services, service_types = self.get_system_state()
                    dt = self.service_if_pool.update(services, service_types)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(nonexistent_srv.resolved_name in dt.added)  # nonexistent_srv added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every added service should be in the list of args
            self.assertTrue(servicename in self.service_if_pool.services_args)
            # service backend has been created
            self.assertTrue(servicename in self.service_if_pool.services.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            nonexistent_srv.shutdown('testing disappearing service')

        # every added service should be in the list of args
        self.assertTrue(servicename in self.service_if_pool.services_args)
        # the backend should STILL be there
        self.assertTrue(servicename in self.service_if_pool.services.keys())
        # Note the service implementation should take care of possible errors in this case

        self.service_if_pool.expose_services([])
        # every withhold service should NOT be in the list of args
        self.assertTrue(servicename not in self.service_if_pool.services_args)
        # service backend should NOT be there any longer
        self.assertTrue(servicename not in self.service_if_pool.services.keys())


#TODO : here we always test the full update => test the diff algorithm as well !

@nose.tools.istest
class TestRosInterface1NoCache(TestRosServiceIfPool):

    @nose.tools.nottest
    def get_system_state(self):
        publishers, subscribers, services = self._master.getSystemState()[2]
        service_types = []  # master misses this API to be consistent
        return services, service_types

    def setUp(self):
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self._master = rospy.get_master()
        self.service_if_pool = RosServiceIfPool()

    def tearDown(self):
        self.service_if_pool = None


# Testing with Connection Cache
@nose.tools.istest
class TestRosInterfaceCache(TestRosServiceIfPool):

    @nose.tools.nottest
    def get_system_state(self):
        # Note getting the system state via this interface, there is no need for callback
        publishers, subscribers, services = self.connection_cache.getSystemState()
        service_types = self.connection_cache.getServiceTypes()
        # handling fallback here since master doesnt have the API

        return services, service_types

    def setUp(self):

        # first we setup our publishers and our node (used by rospy.resolve_name calls to remap topics)
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self._master = rospy.get_master()
        self.service_if_pool = RosServiceIfPool()

        # we need to speed fast enough for the tests to not fail on timeout...
        rospy.set_param('/connection_cache/spin_freq', 2)  # 2 Hz
        self.connection_cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py',
                                                         name='connection_cache',
                                                         remap_args=[('~list', rospy.resolve_name('~connections_list')),
                                                                     (
                                                                     '~diff', rospy.resolve_name('~connections_diff'))])

        # Easier to remap the node topic to the proxy ones, instead of the opposite, since there is no dynamic remapping.
        # However for normal usecase, remapping the proxy handles is preferable.
        try:
            self.connection_cache_proc = self.launch.launch(self.connection_cache_node)
        except roslaunch.RLException as rlexc:
            raise nose.SkipTest("Connection Cache Node not found (part of rocon_python_comms pkg). Skipping test.")

        assert self.connection_cache_proc.is_alive()

        # wait for node to be started
        node_api = None
        with Timeout(5) as t:
            while not t.timed_out and node_api is None:
                node_api = rosnode.get_api_uri(rospy.get_master(), 'connection_cache')

        assert node_api is not None  # make sure the connection cache node is started before moving on.

        self.connection_cache = connection_cache_proxy_create()

    def tearDown(self):
        self.service_if_pool = None

        self.connection_cache_proc.stop()
        while self.connection_cache_proc.is_alive():
            time.sleep(0.2)  # waiting for cache node to die
        assert not self.connection_cache_proc.is_alive()
        time.sleep(1)  # TODO : investigate : we shouldnt need this


if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_service_if_pool_no_cache', 'test_all', TestRosInterface1NoCache)

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_service_if_pool_cache', 'test_all', TestRosInterfaceCache)

