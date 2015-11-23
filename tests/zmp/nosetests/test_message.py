# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import dill
import zmp.message

import nose
from nose.tools import assert_true, assert_false, assert_raises, assert_equal
# TODO : PYTEST ?
# http://pytest.org/latest/contents.html
# https://github.com/ionelmc/pytest-benchmark

# TODO : PYPY
# http://pypy.org/

# Careful : we dont support all fancy methods of the protocol buffer.
# We restrict ourselves to the methods that we are using in zmp.

# Generic initialization tests
def initialize_servicerequest(req):
    assert_equal(req.service, "testservice")
    assert_equal(dill.loads(req.args)[0], "testarg")
    assert_equal(dill.loads(req.kwargs)["testkwarg"], "test")
    assert_true(req.initialized())


def initialize_serviceresponse(resp):
    assert_equal(resp.service, "testservice")
    assert_equal(dill.loads(resp.response), "testresponse")
    assert_equal(resp.type, zmp.message.ServiceResponse.RESPONSE)
    assert_true(resp.initialized())


def default_initialize_servicerequest(req):
    # Although default initialization behaviors are different in protobuf and namedtuple, both should be initialized
    assert_true(req.initialized())


def default_initialize_serviceresponse(resp):
    # Although default initialization behaviors are different in protobuf and namedtuple, both should be initialized
    assert_true(resp.initialized())


# Symmetric Serialize / Parse tests
def oneline_symmetric_serialize_parse_servicerequest(req):
    assert_true(req.initialized())
    # Testing oneline implementation that works for both tuples and protobuf
    finalreq = zmp.message.ServiceRequest_dictparse(req.serialize())
    assert_true(isinstance(finalreq, zmp.message.ServiceRequest))
    assert_true(finalreq.initialized())
    assert_equal(finalreq.service, "testservice")
    assert_equal(dill.loads(finalreq.args)[0], "testarg")
    assert_equal(dill.loads(finalreq.kwargs)["testkwarg"], "test")


def oneline_symmetric_serialize_parse_serviceresponse(resp):
    assert_true(resp.initialized())
    # Testing oneline implementation that works for both tuples and protobuf
    finalresp = zmp.message.ServiceResponse_dictparse(resp.serialize())
    assert_true(isinstance(finalresp, zmp.message.ServiceResponse))
    assert_true(finalresp.initialized())
    assert_equal(finalresp.type, zmp.message.ServiceResponse.RESPONSE)
    assert_equal(finalresp.service, "testservice")
    assert_equal(dill.loads(finalresp.response), "testresponse")

#PROTOBUF Implementation test - default
def test_initialize_servicerequest_protobuf():
    # Test Initialization
    req = zmp.message.ServiceRequest(
        service="testservice",
        args=dill.dumps(("testarg",)),
        kwargs=dill.dumps({'testkwarg': 'test'}),
    )
    # Check we have desired implementation
    assert_true(isinstance(req, zmp.message.ServiceRequestImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(req, zmp.message.ServiceRequest))
    # run actual test
    initialize_servicerequest(req)


def test_initialize_serviceresponse_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceResponse(
        type=zmp.message.ServiceResponse.RESPONSE,
        service="testservice",
        response=dill.dumps("testresponse"),
    )
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceResponseImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceResponse))
    # run actual test
    initialize_serviceresponse(resp)


def test_default_initialize_servicerequest_protobuf():
    # Test Initialization
    req = zmp.message.ServiceRequest()
    # Check we have desired implementation
    assert_true(isinstance(req, zmp.message.ServiceRequestImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(req, zmp.message.ServiceRequest))
    # run actual test
    default_initialize_servicerequest(req)


def test_default_initialize_serviceresponse_protobuf():
    # Test Initialization
    req = zmp.message.ServiceResponse()
    # Check we have desired implementation
    assert_true(isinstance(req, zmp.message.ServiceResponseImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(req, zmp.message.ServiceResponse))
    # run actual test
    default_initialize_serviceresponse(req)


def test_symmetric_serialize_parse_servicerequest_protobuf():
    # Test Initialization
    req = zmp.message.ServiceRequest(
        service="testservice",
        args=dill.dumps(("testarg",)),
        kwargs=dill.dumps({'testkwarg': 'test'}),
    )
    # Check we have desired implementation
    assert_true(isinstance(req, zmp.message.ServiceRequestImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(req, zmp.message.ServiceRequest))

    assert_true(req.initialized())

    # assert parse modifies the current message
    parsedreq = zmp.message.ServiceRequest()
    parsedreq._parse(req.serialize())
    assert_true(isinstance(parsedreq, zmp.message.ServiceRequest))
    assert_true(parsedreq.initialized())
    assert_equal(parsedreq.service, "testservice")
    assert_equal(dill.loads(parsedreq.args)[0], "testarg")
    assert_equal(dill.loads(parsedreq.kwargs)["testkwarg"], "test")

    # assert parse returns the modified list
    parsedreq_oneline = zmp.message.ServiceRequest()._parse(req.serialize())
    assert_true(isinstance(parsedreq_oneline, zmp.message.ServiceRequest))
    assert_true(parsedreq_oneline.initialized())
    assert_equal(parsedreq_oneline.service, "testservice")
    assert_equal(dill.loads(parsedreq_oneline.args)[0], "testarg")
    assert_equal(dill.loads(parsedreq_oneline.kwargs)["testkwarg"], "test")

    # run actual tuple-compatible online test
    oneline_symmetric_serialize_parse_servicerequest(req)


def test_symmetric_serialize_parse_serviceresponse_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceResponse(
        type=zmp.message.ServiceResponse.RESPONSE,
        service="testservice",
        response=dill.dumps("testresponse"),
    )
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceResponseImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceResponse))

    assert_true(resp.initialized())

    # assert parse modifies the current message
    parsedresp = zmp.message.ServiceResponse()
    parsedresp._parse(resp.serialize())
    assert_true(isinstance(parsedresp, zmp.message.ServiceResponse))
    assert_true(parsedresp.initialized())
    assert_equal(parsedresp.type, zmp.message.ServiceResponse.RESPONSE)
    assert_equal(parsedresp.service, "testservice")
    assert_equal(dill.loads(parsedresp.response), "testresponse")

    # assert parse returns the modified list
    parsedresp_oneline = zmp.message.ServiceResponse()._parse(resp.serialize())
    assert_true(isinstance(parsedresp_oneline, zmp.message.ServiceResponse))
    assert_true(parsedresp_oneline.initialized())
    assert_equal(parsedresp_oneline.type, zmp.message.ServiceResponse.RESPONSE)
    assert_equal(parsedresp_oneline.service, "testservice")
    assert_equal(dill.loads(parsedresp_oneline.response), "testresponse")

    # run actual tuple-compatible online test
    oneline_symmetric_serialize_parse_serviceresponse(resp)

if __name__ == '__main__':

    import nose
    nose.runmodule()
