# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import pickle
import zmp.message

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
# import actual test methods from around here
from test_message import (
    initialize_servicerequest,
    initialize_serviceresponse,
    default_initialize_servicerequest,
    default_initialize_serviceresponse,
    oneline_symmetric_serialize_parse_servicerequest,
    oneline_symmetric_serialize_parse_serviceresponse
)


import nose
from nose.tools import assert_true, assert_false, assert_raises, assert_equal
# TODO : PYTEST ?
# http://pytest.org/latest/contents.html
# https://github.com/ionelmc/pytest-benchmark

# TODO : PYPY
# http://pypy.org/


class TestMessageTupleFallback(object):
    def setUp(self):
        # Forcing tuple implementation
        zmp.message.force_namedtuple_implementation()

    def tearDown(self):
        # Setting back default implementation
        zmp.message.force_protobuf_implementation()

    def test_initialize_servicerequest_namedtuple(self):
        #Test Initialization
        req = zmp.message.ServiceRequest(
            service="testservice",
            args=pickle.dumps(("testarg",)),
            kwargs=pickle.dumps({'testkwarg': 'test'}),
        )
        # Check we have desired implementation
        assert_true(isinstance(req, zmp.message.ServiceRequestNTImpl))
        # Check it is an instance of Dynamic Functional Facade
        assert_true(isinstance(req, zmp.message.ServiceRequest))
        # run actual test
        initialize_servicerequest(req)


    def test_initialize_serviceresponse_namedtuple(self):
        #Test Initialization
        resp = zmp.message.ServiceResponse(
            type=zmp.message.ServiceResponse.RESPONSE,
            service="testservice",
            response=pickle.dumps("testresponse")
        )
        # Check we have desired implementation
        assert_true(isinstance(resp, zmp.message.ServiceResponseNTImpl))
        # Check it is an instance of Dynamic Functional Facade
        assert_true(isinstance(resp, zmp.message.ServiceResponse))
        # run actual test
        initialize_serviceresponse(resp)


    def test_default_initialize_servicerequest_namedtuple(self):
        # Test Initialization
        with assert_raises(TypeError) as cm:
            req = zmp.message.ServiceRequest()

        req = zmp.message.ServiceRequest(service=None, args=None, kwargs=None)
        # Check we have desired implementation
        assert_true(isinstance(req, zmp.message.ServiceRequestNTImpl))
        # Check it is an instance of Dynamic Functional Facade
        assert_true(isinstance(req, zmp.message.ServiceRequest))
        # run actual test
        default_initialize_servicerequest(req)


    def test_default_initialize_serviceresponse_namedtuple(self):
        # Test Initialization
        with assert_raises(TypeError) as cm:
            resp = zmp.message.ServiceResponse()

        resp = zmp.message.ServiceResponse(type=zmp.message.ServiceResponse.ERROR, service=None, response=None)
        # Check we have desired implementation
        assert_true(isinstance(resp, zmp.message.ServiceResponseNTImpl))
        # Check it is an instance of Dynamic Functional Facade
        assert_true(isinstance(resp, zmp.message.ServiceResponse))
        # run actual test
        default_initialize_serviceresponse(resp)


    def test_symmetric_serialize_parse_servicerequest_namedtuple(self):
        # Test Initialization
        req = zmp.message.ServiceRequest(
            service="testservice",
            args=pickle.dumps(("testarg",)),
            kwargs=pickle.dumps({'testkwarg': 'test'}),
        )
        # Check we have desired implementation
        assert_true(isinstance(req, zmp.message.ServiceRequestNTImpl))
        # Check it is an instance of Dynamic Functional Facade
        assert_true(isinstance(req, zmp.message.ServiceRequest))

        assert_true(req.initialized())

        # run actual protobuf-compatible online test
        oneline_symmetric_serialize_parse_servicerequest(req)


    def test_symmetric_serialize_parse_serviceresponse_namedtuple(self):
        #Test Initialization
        resp = zmp.message.ServiceResponse(
            type=zmp.message.ServiceResponse.RESPONSE,
            service="testservice",
            response=pickle.dumps("testresponse")
        )
        # Check we have desired implementation
        assert_true(isinstance(resp, zmp.message.ServiceResponseNTImpl))
        # Check it is an instance of Dynamic Functional Facade
        assert_true(isinstance(resp, zmp.message.ServiceResponse))

        assert_true(resp.initialized())

        # run actual protobuf-compatible online test
        oneline_symmetric_serialize_parse_serviceresponse(resp)


if __name__ == '__main__':

    import nose
    nose.runmodule()
