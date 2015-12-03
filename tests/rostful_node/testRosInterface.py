#!/usr/bin/env python
from __future__ import absolute_import


import sys
import os
# to be able to run from source
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

# To simulate ROS env setup
from rostful_node import rostest_nose
# ROS imports should now work from ROS or from python (without ROS env setup)
import rospy
import roslaunch
from std_msgs.msg import String, Empty

# Unit test import
from rostful_node.rosinterface import RosInterface

import unittest

# test node process not setup by default (rostest dont need it here)
empty_srv_process = None
trigger_srv_process = None


# This should have the same effect as the <name>.test file for rostest. Should be used only by nose ( or other python test tool )
def setup_module():
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match
        global empty_srv_process, trigger_srv_process
        empty_srv_node = roslaunch.core.Node('pyros', 'emptyService.py', name='empty_service')
        trigger_srv_node = roslaunch.core.Node('pyros', 'triggerService.py', name='trigger_service')
        empty_srv_process = launch.launch(empty_srv_node)
        trigger_srv_process = launch.launch(trigger_srv_node)


def teardown_module():
    if not rostest_nose.is_rostest_enabled():
        # finishing all process are finished
        if empty_srv_process is not None:
            empty_srv_process.stop()
        if trigger_srv_process is not None:
            trigger_srv_process.stop()

        rostest_nose.rostest_nose_teardown_module()


class TestRosInterface(unittest.TestCase):
    def setUp(self):
        rospy.init_node('ros_interface_test')
        self.strpub = rospy.Publisher('/test/string', String, queue_size=1)
        self.emppub = rospy.Publisher('/test/empty', Empty, queue_size=1)
        
        self.interface = RosInterface(run_watcher=False)

    def tearDown(self):
        self.interface = None

    ##
    # Test basic topic adding functionality for a topic which already exists in
    # the ros environment.
    def test_topic_add_basic_existing(self):
        topicname = '/test/string'
        self.interface.add_topic(topicname)
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # added a single instance of the topic, so the number of the topics
        # existing should be incremented by one (because we consider the fact
        # that when the entire system is running, two additional topics will be
        # created by the subscriber proxy)
        self.assertEqual(-1, self.interface.topics_args[topicname])
        # the topic already exists in the environment, so it should not be in
        # the waiting list
        self.assertTrue(topicname not in self.interface.topics_waiting)
        # make sure the topic backend has been created
        self.assertTrue(topicname in self.interface.topics)

    ##
    # Test basic topic adding functionality for a topic which does not yet exist
    # in the ros environment
    def test_topic_add_basic_nonexistent(self):
        topicname = '/test/nonexistent'
        self.interface.add_topic(topicname)
        # every added topic should be in the list of args
        self.assertTrue(topicname in self.interface.topics_args)
        # we added the topic, but it didn't exist yet, so the count should be -2
        self.assertEqual(-2, self.interface.topics_args[topicname])
        # topic does not exist in the environment, so it should be in the waiting list
        self.assertTrue(topicname in self.interface.topics_waiting)
        # the backend should not have been created
        self.assertTrue(topicname not in self.interface.topics)

        # create the publisher and then try adding the topic again, simulating
        # it coming online.
        nonexistent_pub = rospy.Publisher(topicname, Empty, queue_size=1)
        self.interface.add_topic(topicname)

        self.assertTrue(topicname in self.interface.topics_args)
        self.assertEqual(-1, self.interface.topics_args[topicname])
        self.assertTrue(topicname in self.interface.topics)
        self.assertTrue(topicname not in self.interface.topics_waiting)

    @unittest.expectedFailure
    def test_topic_del(self):
        print("topic del")
        self.assertTrue(False)

    @unittest.expectedFailure
    def test_service_add(self):
        print("service del")
        self.assertTrue(False)

    @unittest.expectedFailure
    def test_service_del(self):
        print("service del")
        self.assertTrue(False)

    ##
    # Ensure that no crash occurs when the expose topics function is called by a
    # reconfigure request which requires the deletion of some topics from the
    # topic arguments that already exist.
    def test_expose_topics_deletion_crash(self):
        self.interface.add_topic('/test/string')
        self.interface.add_topic('/test/empty')
        
        self.interface.expose_topics(['/test/string'])
        # ensure all the lists/dicts no longer contain the deleted topic
        self.assertTrue('/test/empty' not in self.interface.topics_args)
        self.assertTrue('/test/empty' not in self.interface.topics)
        self.assertTrue('/test/empty' not in self.interface.topics_waiting)
        # ensure all relevant lists have the remaining one
        self.assertTrue('/test/string' in self.interface.topics_args)
        self.assertTrue('/test/string' in self.interface.topics)

    ##
    # Ensure that no crash occurs when the expose services function is called by
    # a reconfigure request which requires the deletion of some services from the
    # service arguments that already exist.
    def test_expose_services_deletion_crash(self):
        self.interface.add_service('/test/empsrv')
        self.interface.add_service('/test/trgsrv')
        self.interface.expose_services(['/test/empsrv'])
        # ensure all the lists/dicts no longer contain the deleted service
        self.assertTrue('/test/trgsrv' not in self.interface.services_args)
        self.assertTrue('/test/trgsrv' not in self.interface.services)
        self.assertTrue('/test/trgsrv' not in self.interface.services_waiting)

        # ensure all relevant lists have the remaining one
        self.assertTrue('/test/empsrv' in self.interface.services_args)
        self.assertTrue('/test/empsrv' in self.interface.services)

    # def test_reconfigure_topic(self):
    #     config = {'services': [], 'actions': []}
    #     config['topics'] = "['/test/1', '/test/2', '/test/3', '/testreg/.*', '/.*/regtest']"
    #     self.interface.reconfigure(config, 1)
    #     expected_dict = {'/test/1': -2, '/test/2': -2, '/test/3': -2, '/testreg/.*': -2, '/.*/regtest': -2}
    #     expected_list = ['/test/1', '/test/2', '/test/3', '/testreg/.*', '/.*/regtest']
    #     self.assertEqual(self.interface.topics_args, expected_dict)
    #     # don't care about ordering, just contents
    #     self.assertEqual(sorted(self.interface.topics_waiting), sorted(expected_list))

    # @unittest.expectedFailure
    # def test_reconfigure_service(self):
    #     print("recon service")

    # ##
    # # Ensure that if there are malformed strings in the reconfigure request they are ignored
    # def test_reconfigure_malformed(self):
    #     config = {'actions': []}
    #     config['topics'] = "['/test(/1bad', '/test/)bad', '/test/good']"
    #     config['services'] = "['/test(/1bad', '/test/)bad', '/test/good']"
    #     self.interface.reconfigure(config, 1)
    #     assert self.interface.topics_args == {}
    #     assert self.interface.services_args == []
        
if __name__ == '__main__':

    # Note : Tests should be able to run with nosetests, or rostest ( which will launch nosetest here )
    rostest_nose.rostest_or_nose_main('test_ros_interface', 'test_all', TestRosInterface)

