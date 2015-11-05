# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import time
import mockinterface.mockframework as mockframework

from nose.tools import assert_true, assert_false, assert_raises


### TESTING NODE CREATION / TERMINATION ###
def test_node_creation_termination():
    n1 = mockframework.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())

### TODO : more testing in case of crash in process, exception, etc.


### TESTING PARAMETERS ###


### TESTING TOPIC COMMUNICATION ###


### TESTING SERVICE COMMUNICATION ###
def test_service_comm_to_sub():
    n1 = mockframework.Node(name="HNode")
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())

    def hw(msg):
        return "Hello! I am " + mockframework.current_node().name if msg == "Hello" else "..."

    helloworld = mockframework.Service("HelloWorld", hw)
    n1.provide(helloworld)

    assert_true(helloworld.call("Hello") == "Hello! I am HNode")
    assert_true(helloworld.call("Hallo") == "...")

    n1.shutdown()
    assert_false(n1.is_alive())





if __name__ == '__main__':

    import nose
    nose.runmodule()
