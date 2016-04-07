# -*- coding: utf-8 -*-
from __future__ import absolute_import

# To allow python to run these tests as main script
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import time
import multiprocessing
import zmp
import inspect

import nose
from nose.tools import assert_true, assert_false, assert_raises, assert_equal, nottest, istest
# TODO : PYTEST ?
# http://pytest.org/latest/contents.html
# https://github.com/ionelmc/pytest-benchmark

# TODO : PYPY
# http://pypy.org/


@istest
# IPC protocol
# Node as fixture to guarantee cleanup
# Better to have IPC as main class as it is simpler and easier to test than Socket.
class TestMockHWNodeIPC(object):
    __test__ = True

    class HWNode(zmp.Node):
        def __init__(self, name):
            super(TestMockHWNodeIPC.HWNode, self).__init__(name)
            self.magic_number = 666
            # TODO : improvement : autodetect class own methods
            # TODO : assert static ?
            self.provides(self.helloworld)
            self.provides(self.breakworld)
            self.provides(self.add)
            self.provides(self.getlucky)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def helloworld(msg):
            return "Hello! I am " + zmp.current_node().name if msg == "Hello" else "..."

        @staticmethod
        def breakworld(msg):
            raise Exception("Excepting Not Exceptionnally")

        @staticmethod
        def add(a, b):
            return a+b

        def getlucky(self):
            return self.magic_number

    def setUp(self):
        # services is already setup globally
        self.hwnode = TestMockHWNodeIPC.HWNode(name="HNode")
        self.hwnodeextra = TestMockHWNodeIPC.HWNode(name="HNodeExtra")

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
        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld")
        assert_true(helloworld is None)  # service not provided until node starts

        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld")
        assert_true(helloworld is None)

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover_timeout(self):
        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld")
        assert_true(helloworld is None)  # service not provided until node starts

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 1)  # check timeout actually times out
        assert_true(helloworld is None)

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 1, 2)
        assert_true(helloworld is None)

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover_multiple_stack(self):
        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld")
        assert_true(helloworld is None)  # service not provided until node starts

        # Start two nodes - stack process
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5, 2)  # we wait until we get 2 providers ( or timeout )
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 2)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld")  # we should have right away 1 provider only
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_discover_multiple_queue(self):
        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld")
        assert_true(helloworld is None)  # service not provided until node starts

        # Start two nodes queue process
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5, 2)   # we wait until we get 2 providers ( or timeout )
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 2)

        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5)  # we wait a bit to let it time to start
        assert_false(helloworld is None)
        assert_equal(len(helloworld.providers), 1)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_comm_to_sub(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5)
        assert_true(helloworld is not None)  # to make sure we get a service provided
        resp = helloworld.call(args=("Hello",))
        print "Hello -> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")
        resp = helloworld.call(args=("Hallo",))
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "...")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_comm_to_double_sub(self):
        print("\n" + inspect.currentframe().f_code.co_name)

        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        assert_false(self.hwnodeextra.is_alive())
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5, 2)  # make sure we get both providers. we need them.
        assert_true(helloworld is not None)  # to make sure we get a service provided
        assert_equal(len(helloworld.providers), 2)
        resp = helloworld.call(args=("Hello",))
        print "Hello -> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode" or resp == "Hello! I am HNodeExtra")
        resp = helloworld.call(args=("Hallo",))
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "...")

        resp = helloworld.call(args=("Hello",), node=self.hwnode.name)
        print "Hello -HNode-> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")
        resp = helloworld.call(args=("Hello",), node=self.hwnodeextra.name)
        print "Hello -HNodeExtra-> {0}".format(resp)
        assert_true(resp == "Hello! I am HNodeExtra")

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())
        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_double_comm_to_sub(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering helloworld Service..."
        helloworld = zmp.discover("helloworld", 5)
        assert_true(helloworld is not None)  # to make sure we get a service provided

        def callit():
            hw = zmp.discover("helloworld", 5)
            return hw.call(args=("Hello",))

        c = multiprocessing.Process(name="Client", target=callit)
        assert_false(c.is_alive())
        c.start()
        assert_true(c.is_alive())

        resp = helloworld.call(args=("Hello",))
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")
        resp = helloworld.call(args=("Hallo",))
        print "Hallo -> {0}".format(resp)
        assert_true(resp == "...")
        resp = helloworld.call(args=("Hello",), node=self.hwnode.name)
        print "Hello -HNode-> {0}".format(resp)
        assert_true(resp == "Hello! I am HNode")

        c.join()
        assert_false(c.is_alive())

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_comm_to_sub_self(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.magic_number = 42
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering getlucky Service..."
        getlucky = zmp.discover("getlucky", 5)
        assert_true(getlucky is not None)  # to make sure we get a service provided
        resp = getlucky.call()
        print "42 ? -> {0}".format(resp)
        assert_true(resp == 42)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_comm_to_double_sub_self(self):
        print("\n" + inspect.currentframe().f_code.co_name)

        assert_false(self.hwnode.is_alive())
        self.hwnode.magic_number = 42
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        assert_false(self.hwnodeextra.is_alive())
        self.hwnodeextra.magic_number = 79
        self.hwnodeextra.start()
        assert_true(self.hwnodeextra.is_alive())

        print "Discovering getlucky Service..."
        getlucky = zmp.discover("getlucky", 5, 2)  # make sure we get both providers. we need them.
        assert_true(getlucky is not None)  # to make sure we get a service provided
        assert_equal(len(getlucky.providers), 2)
        resp = getlucky.call()
        print "42 || 79 ? -> {0}".format(resp)
        assert_true(resp == 42 or resp == 79)

        resp = getlucky.call(node=self.hwnode.name)
        print "42 ? -HNode-> {0}".format(resp)
        assert_true(resp == 42)
        resp = getlucky.call(node=self.hwnodeextra.name)
        print "79 ? -HNodeExtra-> {0}".format(resp)
        assert_true(resp == 79)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())
        self.hwnodeextra.shutdown()
        assert_false(self.hwnodeextra.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_double_comm_to_sub_self(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.magic_number = 42
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering getlucky Service..."
        getlucky = zmp.discover("getlucky", 5)
        assert_true(getlucky is not None)  # to make sure we get a service provided

        def callit():
            getlucky = zmp.discover("getlucky", 5)
            return getlucky.call()

        c = multiprocessing.Process(name="Client", target=callit)
        assert_false(c.is_alive())
        c.start()
        assert_true(c.is_alive())

        resp = getlucky.call()
        print "42 ? -> {0}".format(resp)
        assert_true(resp == 42)
        resp = getlucky.call(node=self.hwnode.name)
        print "42 ? -HNode-> {0}".format(resp)
        assert_true(resp == 42)

        c.join()
        assert_false(c.is_alive())

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())



    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_except_from_sub(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering breakworld Service..."
        breakworld = zmp.discover("breakworld", 5)
        assert_true(breakworld is not None)  # to make sure we get a service provided
        with assert_raises(Exception) as cm:
            resp = breakworld.call(args=("Hello",))

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_except_from_node_no_service(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering breakworld Service..."
        breakworld = zmp.discover("breakworld", 5)
        assert_true(breakworld is not None)  # to make sure we get a service provided

        # messing around even if we should not
        breakworld.name = "NOT_EXISTING"

        with assert_raises(zmp.UnknownServiceException) as cm:
            resp = breakworld.call(args=("Hello",))

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())

    # TODO : check mo exception cases

    # @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
    def test_service_params_comm_to_sub(self):

        print("\n" + inspect.currentframe().f_code.co_name)
        assert_false(self.hwnode.is_alive())
        self.hwnode.start()
        assert_true(self.hwnode.is_alive())

        print "Discovering add Service..."
        add = zmp.discover("add", 5)
        assert_true(add is not None)  # to make sure we get a service provided

        resp = add.call(args=(17, 25))
        print " 17 + 25 -> {0}".format(resp)
        assert_true(resp == 17+25)

        self.hwnode.shutdown()
        assert_false(self.hwnode.is_alive())


# Node as fixture to guarantee cleanup
# TCP protocol
@istest
class TestMockHWNodeSocket(TestMockHWNodeIPC):
    __test__ = True

    class HWNode(zmp.Node):
        def __init__(self, name, socket_bind):
            super(TestMockHWNodeSocket.HWNode, self).__init__(name, socket_bind)
            self.magic_number = 999
            # TODO : improvement : autodetect class own methods
            # TODO : assert static ?
            self.provides(self.helloworld)
            self.provides(self.breakworld)
            self.provides(self.add)
            self.provides(self.getlucky)

        @staticmethod  # TODO : verify : is it true that a service is always a static method ( execution does not depend on instance <=> process local data ) ?
        def helloworld(msg):
            return "Hello! I am " + zmp.current_node().name if msg == "Hello" else "..."

        @staticmethod
        def breakworld(msg):
            raise Exception("Excepting Not Exceptionnally")

        @staticmethod
        def add(a, b):
            return a+b

        def getlucky(self):
            return self.magic_number

    def setUp(self):
        # services is already setup globally
        self.hwnode = TestMockHWNodeSocket.HWNode(name="HNode", socket_bind="tcp://127.0.0.1:4242")
        self.hwnodeextra = TestMockHWNodeSocket.HWNode(name="HNodeExtra", socket_bind="tcp://127.0.0.1:4243")

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

    #TODO : verify all tests are run again with proper fixture

if __name__ == '__main__':

    import nose
    nose.runmodule()
