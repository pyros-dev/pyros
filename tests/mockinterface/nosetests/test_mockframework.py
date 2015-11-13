# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import time
import multiprocessing
import mockinterface.mockframework as mockframework

import nose
from nose.tools import assert_true, assert_false, assert_raises


### TESTING NODE CREATION / TERMINATION ###
def test_node_termination():
    n1 = mockframework.Node()
    assert_false(n1.is_alive())
    n1.shutdown()  # shutdown should have no effect here (if not started, same as noop )
    assert_false(n1.is_alive())

def test_node_creation_termination():
    n1 = mockframework.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())

### TODO : more testing in case of crash in process, exception, signal, etc.


### TESTING PARAMETERS ###


### TESTING TOPIC COMMUNICATION ###


### TESTING SERVICE COMMUNICATION ###
def test_service_discover():

    class HWNode(mockframework.Node):
        def __init__(self, name):
            super(HWNode, self).__init__(name)
            # TODO : improvement : autodetect class own methods
            self.provides("HelloWorld", self.hw)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def hw(msg):
            return "Hello! I am " + mockframework.current_node().name if msg == "Hello" else "..."

    testing = True
    lcb = "data"
    helloworld = mockframework.discover("HelloWorld")
    assert_true(helloworld is None)

    n1 = HWNode(name="HNode")
    assert_false(n1.is_alive())

    helloworld = mockframework.discover("HelloWorld")
    assert_true(helloworld is None)  # service not provided until node starts

    n1.start()
    assert_true(n1.is_alive())

    helloworld = mockframework.discover("HelloWorld", 5)  # we wait a bit to get something
    assert_false(helloworld is None)

    n1.shutdown()
    assert_false(n1.is_alive())

    helloworld = mockframework.discover("HelloWorld")
    assert_true(helloworld is None)

@nose.SkipTest
def test_service_comm_to_sub():

    class HWNode(mockframework.Node):
        def __init__(self, name):
            super(HWNode, self).__init__(name)
            # TODO : improvement : autodetect class own methods
            self.provides("HelloWorld", self.hw)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def hw(msg):
            return "Hello! I am " + mockframework.current_node().name if msg == "Hello" else "..."

    testing = True
    lcb = "data"

    n1 = HWNode(name="HNode")
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())

    helloworld = mockframework.discover("HelloWorld", 5)
    assert_true(helloworld is not None)  # to make sure we get a service provided
    assert_true(helloworld.call("Hello") == "Hello! I am HNode")
    assert_true(helloworld.call("Hallo") == "...")

    n1.shutdown()
    assert_false(n1.is_alive())

@nose.SkipTest
def test_service_comm_to_double_sub():

    class HWNode(mockframework.Node):
        def __init__(self, name):
            super(HWNode, self).__init__(name)
            # TODO : improvement : autodetect class own methods
            self.provides("HelloWorld", self.hw)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def hw(msg):
            return "Hello! I am " + mockframework.current_node().name if msg == "Hello" else "..."

    testing = True
    lcb = "data"

    n1 = HWNode(name="HNode1")
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())

    n2 = HWNode(name="HNode2")
    assert_false(n2.is_alive())
    n2.start()
    assert_true(n2.is_alive())

    helloworld = mockframework.discover("HelloWorld", 5)
    assert_true(helloworld is not None)  # to make sure we get a service provided
    assert_true(helloworld.call("Hello") == "Hello! I am HNode1" or helloworld.call("Hello") == "Hello! I am HNode2")
    assert_true(helloworld.call("Hallo") == "...")

    assert_true(helloworld.call("Hello", n1) == "Hello! I am HNode1")
    assert_true(helloworld.call("Hello", n2) == "Hello! I am HNode2")

    n1.shutdown()
    assert_false(n1.is_alive())


@nose.SkipTest
def test_service_double_comm_to_sub():

    class HWNode(mockframework.Node):
        def __init__(self, name):
            super(HWNode, self).__init__(name)
            # TODO : improvement : autodetect class own methods
            self.provides("HelloWorld", self.hw)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def hw(msg):
            return "Hello! I am " + mockframework.current_node().name if msg == "Hello" else "..."

    testing = True
    lcb = "data"

    n1 = HWNode(name="HNode1")
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())

    def callit():
        hw = mockframework.discover("HelloWorld", 5)
        return hw.call("Hello")

    c = multiprocessing.Process(name="Client", target=callit)
    assert_false(c.is_alive())
    c.start()
    assert_true(c.is_alive())

    helloworld = mockframework.discover("HelloWorld", 5)
    assert_true(helloworld is not None)  # to make sure we get a service provided
    assert_true(helloworld.call("Hello") == "Hello! I am HNode1")
    assert_true(helloworld.call("Hallo") == "...")
    assert_true(helloworld.call("Hello", n1) == "Hello! I am HNode1")

    c.join()
    assert_false(c.is_alive())

    n1.shutdown()
    assert_false(n1.is_alive())


if __name__ == '__main__':

    import nose
    nose.runmodule()
