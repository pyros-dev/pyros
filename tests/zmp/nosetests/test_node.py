# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import time
import zmp

import nose
from nose.tools import assert_true, assert_false, assert_raises, assert_equal
# TODO : PYTEST ?
# http://pytest.org/latest/contents.html
# https://github.com/ionelmc/pytest-benchmark

# TODO : PYPY
# http://pypy.org/

### TESTING NODE CREATION / TERMINATION ###
# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
def test_node_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.shutdown()  # shutdown should have no effect here (if not started, same as noop )
    assert_false(n1.is_alive())


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
def test_node_creation_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
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
def test_node_creation_double_termination():
    n1 = zmp.Node()
    assert_false(n1.is_alive())
    n1.start()
    assert_true(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())
    n1.shutdown()
    assert_false(n1.is_alive())

### TODO : more testing in case of crash in process, exception, signal, etc.

if __name__ == '__main__':

    import nose
    nose.runmodule()
