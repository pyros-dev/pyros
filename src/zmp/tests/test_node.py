# -*- coding: utf-8 -*-
from __future__ import absolute_import

# To allow python to run these tests as main script
import functools
import sys
import os
import threading

import types
from random import randint

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import time
import zmp

import nose
from nose.tools import timed, assert_true, assert_false, assert_raises, assert_equal
# TODO : PYTEST ?
# http://pytest.org/latest/contents.html
# https://github.com/ionelmc/pytest-benchmark

# TODO : PYPY
# http://pypy.org/

# TODO : Test Node exception : correctly transmitted, node still keeps spinning...

### TESTING NODE CREATION / TERMINATION ###
# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_node_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.shutdown()  # shutdown should have no effect here (if not started, same as noop )
    assert_false(n1.is_alive())


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_node_creation_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_node_double_creation_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.start()  # this shuts down and restart the node
    assert_true(n1.is_alive())

    n1.shutdown()
    assert_false(n1.is_alive())


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_node_creation_double_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())

# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_node_as_context_manager():
    with zmp.Node() as n1:  # this will __init__ and __enter__
        assert_true(n1.is_alive())
    assert_true(not n1.is_alive())

# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_node_running_as_context_manager():
    n1 = zmp.Node()
    n1.start()
    with n1:  # hooking to an already started node
        assert_true(n1.is_alive())
    assert_true(not n1.is_alive())


def test_update_rate():
    """
    Testing that the update methods get a correct timedelta
    :return:
    """
    # TODO : investigate if node multiprocessing plugin would help simplify this
    # playing with list to pass a reference to this
    testing_last_update = [time.time()]
    testing_time_delta = []
    acceptable_timedelta = []

    def testing_update(self, timedelta, last_update, time_delta, ok_timedelta):
        time_delta.append(time.time() - last_update[-1])
        last_update.append(time.time())

        # if the time delta measured in test and the one passed as argument differ
        # too much, one time, test is failed
        if abs(time_delta[-1] - timedelta) > 0.005:
            ok_timedelta.append(False)
        else:
            ok_timedelta.append(True)

        # spin like crazy, loads CPU for a bit, and eventually exits.
        # We re here trying to disturb the update rate
        while True:
            if randint(0, 10000) == 42:
                break



    # hack to dynamically change the update method
    testing_update_onearg = functools.partial(testing_update,
                                        last_update=testing_last_update,
                                        time_delta=testing_time_delta,
                                        ok_timedelta=acceptable_timedelta)

    n1 = zmp.Node()
    n1.update = types.MethodType(testing_update_onearg, n1)

    assert_false(n1.is_alive())

    # Starting the node in the same thread, to be able to test simply by shared memory.
    # TODO : A Node that can choose process or thread run ( on start() instead of init() maybe ? )
    runthread = threading.Thread(target=n1.run)
    runthread.daemon = True  # to kill this when test is finished
    runthread.start()

    # sleep here for a while
    time.sleep(10)

    # removing init time only used for delta computation
    testing_last_update.pop(0)
    # Check time vars modified by update
    for i in range(0, len(testing_last_update)):
        print("update : {u} | delta: {d} | accept : {a}".format(
            u=testing_last_update[i],
            d=testing_time_delta[i],
            a=acceptable_timedelta[i])
        )

        assert_true(acceptable_timedelta[i])





### TODO : more testing in case of crash in process, exception, signal, etc.

if __name__ == '__main__':

    import nose
    nose.runmodule()
