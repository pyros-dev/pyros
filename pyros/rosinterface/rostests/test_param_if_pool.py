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
from pyros.rosinterface import RosParamIfPool

import rospy
import roslaunch
import rosnode
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv, Trigger

from pyros.rosinterface.rostests import Timeout

# useful test tools
from pyros_utils import rostest_nose
import unittest



# This should have the same effect as the <name>.test file for rostest. Should be used only by nose ( or other python test tool )
# CAREFUL with comments, copy paste mistake are real...
# CAREFUL dont use assertFalse -> easy to miss when reviewing code
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_teardown_module()


# Very basic echo service implementation for tests
def srv_cb(req):
    return req.request


@nose.tools.nottest
class TestRosParamIfPool(unittest.TestCase):
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
        TestRosParamIfPool.launch = roslaunch.scriptapi.ROSLaunch()
        TestRosParamIfPool.launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match
        global empty_srv_process, trigger_srv_process
        empty_srv_node = roslaunch.core.Node('pyros_test', 'emptyService.py', name='empty_service')
        trigger_srv_node = roslaunch.core.Node('pyros_test', 'triggerService.py', name='trigger_service')
        TestRosParamIfPool.empty_srv_process = TestRosParamIfPool.launch.launch(empty_srv_node)
        TestRosParamIfPool.trigger_srv_process = TestRosParamIfPool.launch.launch(trigger_srv_node)

    @classmethod
    def teardown_class(cls):
        # ensuring all process are finished
        if TestRosParamIfPool.empty_srv_process is not None:
            TestRosParamIfPool.empty_srv_process.stop()
        if TestRosParamIfPool.trigger_srv_process is not None:
            TestRosParamIfPool.trigger_srv_process.stop()

    # EXPOSE PARAMS + UPDATE Interface
    def test_param_appear_expose_update(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Simple Normal usecase
        Sequence : APPEAR -> EXPOSE -> UPDATE
        :return:
        """

        paramname = '/test/confirm_param'
        dt = self.param_if_pool.expose_transients_regex([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # param backend has not been created since the update didn't run yet
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # NOTE : We need to wait to make sure the tests nodes are started...
        with Timeout(5) as t:
            while not t.timed_out and paramname not in dt.added:
                params = self.get_system_state()
                dt = self.param_if_pool.update(params)
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        # every exposed param should remain in the list of args ( in case regex match another service )
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # make sure the param backend has been created
        self.assertTrue(paramname in self.param_if_pool.params.keys())

        # cleaning up
        self.param_if_pool.expose_params([])
        # param should not be in the list of args any longer
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # param backend has been removed
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

    def test_param_appear_update_expose(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE?) -> APPEAR -> UPDATE -> EXPOSE (-> UPDATE?)
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())
        # First update should not change state
        params = self.get_system_state()
        dt = self.param_if_pool.update(params)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # create the param and then try exposing the param again, simulating
        # it coming online before expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            with Timeout(5) as t:
                while not t.timed_out and paramname not in self.param_if_pool.params_available:
                    params = self.get_system_state()
                    dt = self.param_if_pool.update(params)
                    self.assertEqual(dt.added, [])  # nothing added (not exposed yet)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            # every added param should be in the list of args
            self.assertTrue(paramname not in self.param_if_pool.params_args)
            # the backend should not have been created
            self.assertTrue(paramname not in self.param_if_pool.params.keys())

            # here we are sure the interface knows the service is available
            # it will be exposed right now
            dt = self.param_if_pool.expose_params([paramname])
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every exposed param should remain in the list of args ( in case regex match another param )
            self.assertTrue(paramname in self.param_if_pool.params_args)
            # make sure the service backend has been created
            self.assertTrue(paramname in self.param_if_pool.params.keys())
        finally:
            rospy.delete_param(paramname)

    def test_param_expose_appear_update(self):
        """
        Test basic param adding functionality for a param which does not yet exist
        in the ros environment ( + corner cases )
        Sequence : (UPDATE? ->) -> EXPOSE -> (UPDATE? ->) APPEAR -> UPDATE
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())
        # First update should not change state
        params = self.get_system_state()
        dt = self.param_if_pool.update(params)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        self.param_if_pool.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())
        params = self.get_system_state()
        dt = self.param_if_pool.update(params)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # make sure the param is STILL in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # make sure the param backend has STILL not been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            with Timeout(5) as t:
                dt = DiffTuple([], [])
                while not t.timed_out and paramname not in dt.added:
                    params = self.get_system_state()
                    dt = self.param_if_pool.update(params)
                    self.assertEqual(dt.removed, [])  # nothing removed
                    time.sleep(0.1)  # to avoid spinning out of control

            self.assertTrue(not t.timed_out)
            self.assertTrue(paramname in dt.added)  # nonexistent_srv added
            # every exposed param should remain in the list of args ( in case regex match another service )
            self.assertTrue(paramname in self.param_if_pool.params_args)
            # make sure the param backend has been created
            self.assertTrue(paramname in self.param_if_pool.params.keys())
        finally:
            rospy.delete_param(paramname)

    def test_param_withhold_update_disappear(self):
        """
        Test param witholding functionality for a param which doesnt exists anymore in
        the ros environment. Normal usecase.
        Sequence : (-> UPDATE ?) -> WITHHOLD -> UPDATE -> DISAPPEAR (-> UPDATE ?)
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        self.param_if_pool.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # service backend has NOT been created yet
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            params = self.get_system_state()
            dt = self.param_if_pool.update(params)
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every withhold param should STILL be in the list of args
            self.assertTrue(paramname in self.param_if_pool.params_args)
            # param backend has been created
            self.assertTrue(paramname in self.param_if_pool.params.keys())

            dt = self.param_if_pool.expose_params([])
            self.assertEqual(dt.added, [])  # nothing added
            self.assertTrue(paramname in dt.removed)  # paramname removed
            # every withhold param should NOT be in the list of args
            self.assertTrue(paramname not in self.param_if_pool.params_args)
            # param backend should be GONE
            self.assertTrue(paramname not in self.param_if_pool.params.keys())

            params = self.get_system_state()
            dt = self.param_if_pool.update(params)
            self.assertEqual(dt.added, [])  # nothing added
            self.assertEqual(dt.removed, [])  # nothing removed
            # every withhold param should STILL NOT be in the list of args
            self.assertTrue(paramname not in self.param_if_pool.params)
            # param backend should be GONE
            self.assertTrue(paramname not in self.param_if_pool.params.keys())
        finally:
            rospy.delete_param(paramname)

        params = self.get_system_state()
        dt = self.param_if_pool.update(params)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nonexistent_srv already removed
        # every withhold service should STILL NOT be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # service backend should be GONE
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

    def test_param_disappear_update_withhold(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Normal usecase
        Sequence : (UPDATE? ->) DISAPPEAR -> UPDATE -> WITHHOLD (-> UPDATE ?)
        :return:
        """
        paramname = '/test/absentparam1'
        # param should not be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        self.param_if_pool.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # param backend has NOT been created yet
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            params = self.get_system_state()
            dt = self.param_if_pool.update(params)
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # param should be in the list of args
            self.assertTrue(paramname in self.param_if_pool.params_args)
            # the backend should have been created
            self.assertTrue(paramname in self.param_if_pool.params.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            rospy.delete_param(paramname)

        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # the backend should STILL be there
        self.assertTrue(paramname in self.param_if_pool.params.keys())
        # Note the param implementation should take care of possible errors in this case

        # wait here until param actually disappear from cache proxy
        with Timeout(5) as t:
            while not t.timed_out and paramname not in dt.removed:
                params = self.get_system_state()
                dt = self.param_if_pool.update(params)
                self.assertEqual(dt.added, [])  # nothing added
                time.sleep(0.1)  # to avoid spinning out of control

        self.assertTrue(not t.timed_out)
        self.assertTrue(paramname in dt.removed)  # nonexistent_srv removed
        # every exposed param should remain in the list of args ( in case regex match another service )
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # make sure the param backend should NOT be there any longer
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # TODO : test that coming back actually works

        self.param_if_pool.expose_params([])
        # every withhold param should NOT be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # param backend has not been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        params = self.get_system_state()
        dt = self.param_if_pool.update(params)
        self.assertEqual(dt.added, [])  # nothing added
        self.assertEqual(dt.removed, [])  # nothing removed
        # every withhold param should NOT be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # make sure the param backend has been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

    def test_param_update_disappear_withhold(self):
        """
        Test param exposing functionality for a param which already exists in
        the ros environment. Simple Normal usecase
        Sequence : UPDATE -> DISAPPEAR -> WITHHOLD
        :return:
        """
        paramname = '/test/absentparam1'
        # every added param should be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # the backend should not have been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        self.param_if_pool.expose_params([paramname])
        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # param backend has not been created
        self.assertTrue(paramname not in self.param_if_pool.params.keys())

        # create the param and then try updating again, simulating
        # it coming online after expose call.
        rospy.set_param(paramname, 'param_value')
        try:
            params = self.get_system_state()
            dt = self.param_if_pool.update(params)
            self.assertTrue(paramname in dt.added)  # paramname added
            self.assertEqual(dt.removed, [])  # nothing removed

            # every added param should be in the list of args
            self.assertTrue(paramname in self.param_if_pool.params_args)
            # param backend has been created
            self.assertTrue(paramname in self.param_if_pool.params.keys())

        # up to here possible sequences should have been already tested by previous tests
        # Now comes our actual disappearance / withholding test
        finally:
            rospy.delete_param(paramname)

        # every added param should be in the list of args
        self.assertTrue(paramname in self.param_if_pool.params_args)
        # the backend should STILL be there
        self.assertTrue(paramname in self.param_if_pool.params.keys())
        # Note the param implementation should take care of possible errors in this case

        self.param_if_pool.expose_params([])
        # every withhold param should NOT be in the list of args
        self.assertTrue(paramname not in self.param_if_pool.params_args)
        # param backend should NOT be there any longer
        self.assertTrue(paramname not in self.param_if_pool.params.keys())


#TODO : here we always test the full update => test the diff algorithm as well !

@nose.tools.istest
class TestRosInterface1NoCache(TestRosParamIfPool):

    @nose.tools.nottest
    def get_system_state(self):
        params = set(rospy.get_param_names())
        return params

    def setUp(self):
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self.param_if_pool = RosParamIfPool()

        # CAREFUL : this is doing a rospy.init_node, and it should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.

    def tearDown(self):
        self.param_if_pool = None


# Testing with Connection Cache
@nose.tools.istest
class TestRosInterfaceCache(TestRosParamIfPool):

    # TMP : until params are interfaced in connection cache as well...
    @nose.tools.nottest
    def get_system_state(self):
        params = set(rospy.get_param_names())
        return params

    def setUp(self):

        # first we setup our publishers and our node (used by rospy.resolve_name calls to remap topics)
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)

        self.param_if_pool = RosParamIfPool()

        # CAREFUL : this is doing a rospy.init_node, and it should be done only once per PROCESS
        # Here we enforce TEST RUN 1<->1 MODULE 1<->1 PROCESS. ROStest style.

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

    def tearDown(self):
        self.param_if_pool = None

        self.connection_cache_proc.stop()
        while self.connection_cache_proc.is_alive():
            time.sleep(0.2)  # waiting for cache node to die
        assert not self.connection_cache_proc.is_alive()
        time.sleep(1)  # TODO : investigate : we shouldnt need this


if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_param_if_pool_no_cache', 'test_all', TestRosInterface1NoCache)

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_param_if_pool_cache', 'test_all', TestRosInterfaceCache)


