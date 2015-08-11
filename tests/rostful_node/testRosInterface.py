#!/usr/bin/env python
import unittest
import sys
import os
import rostest
import rospy
from std_msgs.msg import String, Empty
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))

from rostful_node.ros_interface import RosInterface

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
        self.interface.add_topic('/test/string')
        # every added topic should be in the list of args
        self.assertTrue('/test/string' in self.interface.topics_args)
        # added a single instance of the topic, so the number of the topics
        # existing should be incremented by one (because we consider the fact
        # that when the entire system is running, two additional topics will be
        # created by the subscriber proxy)
        self.assertEqual(-1, self.interface.topics_args['/test/string'])
        # the topic already exists in the environment, so it should not be in
        # the waiting list
        self.assertTrue('/test/string' not in self.interface.topics_waiting)
        # make sure the topic backend has been created
        self.assertTrue('/test/string' in self.interface.topics)

    ##
    # Test basic topic adding functionality for a topic which does not yet exist
    # in the ros environment
    def test_topic_add_basic_nonexistent(self):
        self.interface.add_topic('/test/nonexistent')
        # every added topic should be in the list of args
        self.assertTrue('/test/nonexistent' in self.interface.topics_args)
        # we added the topic, but it didn't exist yet, so the count should be -2
        self.assertEqual(-2, self.interface.topics_args['/test/nonexistent'])
        # topic does not exist in the environment, so it should be in the waiting list
        self.assertTrue('/test/nonexistent' in self.interface.topics_waiting)
        # the backend should not have been created
        self.assertTrue('/test/nonexistent' not in self.interface.topics)

    @unittest.expectedFailure
    def test_topic_del(self):
        print("topic del")

    @unittest.expectedFailure
    def test_service_add(self):
        print("service del")

    @unittest.expectedFailure
    def test_service_del(self):
        print("service del")

    ##
    # Ensure that no crash occurs when the expose topics function is called by a
    # reconfigure request which requires the deletion of some topics from the
    # topic arguments that already exist.
    def test_expose_topics_deletion_crash(self):
        self.interface.add_topic('/test/string')
        self.interface.add_topic('/test/empty')
        
        self.interface.expose_topics(['/test/string'])
        self.assertTrue('/test/empty' not in self.interface.topics_args)
        self.assertTrue('/test/empty' not in self.interface.topics)
        self.assertTrue('/test/empty' not in self.interface.topics_waiting)

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
    rostest.rosrun('test_ros_interface', 'test_all', TestRosInterface)
