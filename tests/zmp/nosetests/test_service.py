# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import time
import multiprocessing
import zmp

import nose
from nose.tools import assert_true, assert_false, assert_raises, assert_equal
# TODO : PYTEST ?
# http://pytest.org/latest/contents.html
# https://github.com/ionelmc/pytest-benchmark

# TODO : PYPY
# http://pypy.org/


# Node as fixture to guarantee cleanup
class TestMockHWNode(object):
    class HWNode(zmp.Node):
        def __init__(self, name, port):
            super(TestMockHWNode.HWNode, self).__init__(name, port=port)
            # TODO : improvement : autodetect class own methods
            # TODO : assert static ?
            self.provides("HelloWorld", self.hw)
            self.provides("BreakWorld", self.bw)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def hw(msg):
            return "Hello! I am " + zmp.current_node().name if msg == "Hello" else "..."

        @staticmethod
        def bw(msg):
            raise Exception("Excepting Not Exceptionnally")

    def setUp(self):
        # services is already setup globally
        self.hwnode = TestMockHWNode.HWNode(name="HNode", port=4242)
        self.hwnodeextra = TestMockHWNode.HWNode(name="HNodeExtra", port=4243)

    def tearDown(self):
        if self.hwnode.is_alive():
            self.hwnode.shutdown(join=True)
        if self.hwnodeextra.is_alive():
            self.hwnodeextra.shutdown(join=True)
        # if it s still alive terminate it.
        if self.hwnode.is_alive():
            self.hwnode.terminate()
        if self.hwnodeextra.is_alive():
            self.hwnodeextra.terminate()

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover(self):
        assert_false(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld")
        assert_true(helloworld is None)  # service not provided until node starts

        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld")
        assert_true(helloworld is None)

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover_timeout(self):
        assert_false(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld")
        assert_true(helloworld is None)  # service not provided until node starts

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 1)  # check timeout actually times out
        assert_true(helloworld is None)

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 1, 2)
        assert_true(helloworld is None)

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover_multiple_stack(self):
        assert_false(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld")
        assert_true(helloworld is None)  # service not provided until node starts

        # Start two nodes - stack process
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5, 2)  # we wait until we get 2 providers ( or timeout )
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 2)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld")  # we should have right away 1 provider only
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover_multiple_queue(self):
        assert_false(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld")
        assert_true(helloworld is None)  # service not provided until node starts

        # Start two nodes queue process
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5, 2)   # we wait until we get 2 providers ( or timeout )
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 2)

        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_comm_to_sub(self):

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5)
        assert_true(helloworld is not None)  # to make sure we get a service provided
        resp = helloworld.call("Hello")
        print "Hello -> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")
        resp = helloworld.call("Hallo")
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "...")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_comm_to_double_sub(self):

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        assert_false(self.hwnodeextra.is_alive())
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5, 2)  # make sure we get both providers. we need them.
        assert_true(helloworld is not None)  # to make sure we get a service provided
        assert_equal(len(helloworld.providers), 2)
        resp = helloworld.call("Hello")
        print "Hello -> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode" or resp == "Hello! I am HNodeExtra")
        resp = helloworld.call("Hallo")
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "...")

        resp = helloworld.call("Hello", self.hwnode.name)
        print "Hello -HNode-> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")
        resp = helloworld.call("Hello", self.hwnodeextra.name)
        print "Hello -HNodeExtra-> {0}".format(resp)
        assert_true(resp == "Hello! I am HNodeExtra")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())
        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_double_comm_to_sub(self):

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering HelloWorld Service..."
        helloworld = zmp.discover("HelloWorld", 5)
        assert_true(helloworld is not None)  # to make sure we get a service provided

        def callit():
            hw = zmp.discover("HelloWorld", 5)
            return hw.call("Hello")

        c = multiprocessing.Process(name="Client", target=callit)
        assert_false(c.is_alive())
        c.start()
        assert_true(c.is_alive())

        resp = helloworld.call("Hello")
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")
        resp = helloworld.call("Hallo")
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "...")
        resp = helloworld.call("Hello")
        print "Hello -HNode-> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")

        c.join()
        assert_false(c.is_alive())

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_except_from_sub(self):

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering BreakWorld Service..."
        breakworld = zmp.discover("BreakWorld", 5)
        assert_true(breakworld is not None)  # to make sure we get a service provided
        with assert_raises(Exception) as cm:
            resp = breakworld.call("Hello")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_except_from_node_no_service(self):

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering BreakWorld Service..."
        breakworld = zmp.discover("BreakWorld", 5)
        assert_true(breakworld is not None)  # to make sure we get a service provided

        # messing around even if we should not
        breakworld.name = "NOT_EXISTING"

        with assert_raises(zmp.UnknownServiceException) as cm:
            resp = breakworld.call("Hello")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_except_from_node_wrong_request(self):

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering BreakWorld Service..."
        breakworld = zmp.discover("BreakWorld", 5)
        assert_true(breakworld is not None)  # to make sure we get a service provided

        # messing around even if we shouldnt
        #TODO

        #with assert_raises(zmp.Node.UnknownRequestTypeException) as cm:
        #    resp = breakworld.call("Hello")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # TODO : check unknown response type

    @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_stress(self):
        # Build a list of subtests
        # run them all !
        pass

if __name__ == '__main__':

    import nose
    nose.runmodule()
