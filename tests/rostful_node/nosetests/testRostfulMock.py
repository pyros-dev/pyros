from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from rostful_node.rostful_mock import RostfulMock

class TestRostfulMock:
    def setUp(self):
        self.mockInstance = RostfulMock()
        self.mockInstance.async_spin()

    def tearDown(self):
        self.mockInstance.async_stop()
        pass

    def test_msg_build(self):
        msg = self.mockInstance.msg_build('fake_connec_name')
        assert isinstance(msg, str)

    def test_echo_topic(self):
        msg = 'testing random_topic'
        self.mockInstance.topic('random_topic', msg)
        print self.mockInstance.topic('random_topic')
        assert msg == self.mockInstance.topic('random_topic')

    def test_echo_service(self):
        msg = 'testing random_service'
        print self.mockInstance.service('random_service', msg)
        assert msg == self.mockInstance.service('random_service', msg)

    #TODO : choose and test proper default values


    #TODO : test pipe interface in another fixture
