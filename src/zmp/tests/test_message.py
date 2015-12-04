# -*- coding: utf-8 -*-
from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import pickle
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
    assert_equal(pickle.loads(req.args)[0], "testarg")
    assert_equal(pickle.loads(req.kwargs)["testkwarg"], "test")
    assert_true(req.initialized())


def initialize_serviceresponse(resp):
    assert_equal(resp.service, "testservice")
    assert_equal(resp.has_field('response'), True)
    assert_equal(resp.has_field('exception'), False)
    assert_equal(pickle.loads(resp.response), "testresponse")
    assert_true(resp.initialized())


def initialize_serviceexception(resp):
    assert_equal(resp.exc_type, "testexception")
    assert_equal(pickle.loads(resp.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(resp.traceback), "testtraceback")
    assert_true(resp.initialized())


def initialize_serviceresponseexception(resp):
    assert_equal(resp.service, "testservice")
    assert_equal(resp.has_field('response'), False)
    assert_equal(resp.has_field('exception'), True)
    assert_equal(resp.exception.exc_type, "testexception")
    assert_equal(pickle.loads(resp.exception.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(resp.exception.traceback), "testtraceback")
    assert_true(resp.initialized())


def default_initialize_servicerequest(req):
    # Although default initialization behaviors are different in protobuf and namedtuple, both should be initialized
    assert_true(req.initialized())


def default_initialize_serviceresponse(resp):
    # Although default initialization behaviors are different in protobuf and namedtuple, both should be initialized
    assert_true(resp.initialized())


def default_initialize_serviceexception(resp):
    # Although default initialization behaviors are different in protobuf and namedtuple, both should be initialized
    assert_true(resp.initialized())


# No point to have a default_initialize_serviceresponseexception here


# Symmetric Serialize / Parse tests
def oneline_symmetric_serialize_parse_servicerequest(req):
    assert_true(req.initialized())
    # Testing oneline implementation that works for both tuples and protobuf
    finalreq = zmp.message.ServiceRequest_dictparse(req.serialize())
    assert_true(isinstance(finalreq, zmp.message.ServiceRequest))
    assert_true(finalreq.initialized())
    assert_equal(finalreq.service, "testservice")
    assert_equal(pickle.loads(finalreq.args)[0], "testarg")
    assert_equal(pickle.loads(finalreq.kwargs)["testkwarg"], "test")


def oneline_symmetric_serialize_parse_serviceresponse(resp):
    assert_true(resp.initialized())
    # Testing oneline implementation that works for both tuples and protobuf
    finalresp = zmp.message.ServiceResponse_dictparse(resp.serialize())
    assert_true(isinstance(finalresp, zmp.message.ServiceResponse))
    assert_true(finalresp.initialized())
    assert_equal(finalresp.service, "testservice")
    assert_equal(finalresp.has_field('response'), True)
    assert_equal(finalresp.has_field('exception'), False)
    assert_equal(pickle.loads(finalresp.response), "testresponse")


def oneline_symmetric_serialize_parse_serviceexception(resp):
    assert_true(resp.initialized())
    # Testing oneline implementation that works for both tuples and protobuf
    finalresp = zmp.message.ServiceException_dictparse(resp.serialize())
    assert_true(isinstance(finalresp, zmp.message.ServiceException))
    assert_true(finalresp.initialized())
    assert_equal(finalresp.exc_type, "testexception")
    assert_equal(pickle.loads(finalresp.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(finalresp.traceback), "testtraceback")


def oneline_symmetric_serialize_parse_serviceresponseexception(resp):
    assert_true(resp.initialized())
    # Testing oneline implementation that works for both tuples and protobuf
    finalresp = zmp.message.ServiceResponse_dictparse(resp.serialize())
    assert_true(isinstance(finalresp, zmp.message.ServiceResponse))
    assert_true(finalresp.initialized())
    assert_equal(finalresp.service, "testservice")
    assert_equal(finalresp.has_field('response'), False)
    assert_equal(finalresp.has_field('exception'), True)
    assert_equal(finalresp.exception.exc_type, "testexception")
    assert_equal(pickle.loads(finalresp.exception.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(finalresp.exception.traceback), "testtraceback")


#PROTOBUF Implementation test - default
def test_initialize_servicerequest_protobuf():
    # Test Initialization
    req = zmp.message.ServiceRequest(
        service="testservice",
        args=pickle.dumps(("testarg",)),
        kwargs=pickle.dumps({'testkwarg': 'test'}),
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
        service="testservice",
        response=pickle.dumps("testresponse"),
    )
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceResponseImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceResponse))
    # run actual test
    initialize_serviceresponse(resp)


def test_initialize_serviceexception_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceException(
        exc_type="testexception",
        exc_value=pickle.dumps("testexceptionvalue"),
        traceback=pickle.dumps("testtraceback"),
    )
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceExceptionImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceException))
    # run actual test
    initialize_serviceexception(resp)


def test_initialize_serviceresponseexception_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceResponse(
        service="testservice",
        exception=zmp.message.ServiceException(
            exc_type="testexception",
            exc_value=pickle.dumps("testexceptionvalue"),
            traceback=pickle.dumps("testtraceback"),
        )
    )
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceResponseImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceResponse))
    # run actual test
    initialize_serviceresponseexception(resp)


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
    resp = zmp.message.ServiceResponse()
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceResponseImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceResponse))
    # run actual test
    default_initialize_serviceresponse(resp)


def test_default_initialize_serviceexception_protobuf():
    # Test Initialization
    exc = zmp.message.ServiceException()
    # Check we have desired implementation
    assert_true(isinstance(exc, zmp.message.ServiceExceptionImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(exc, zmp.message.ServiceException))
    # run actual test
    default_initialize_serviceexception(exc)


# No point to have a test_default_initialize_serviceresponseexception_protobuf here


def test_symmetric_serialize_parse_servicerequest_protobuf():
    # Test Initialization
    req = zmp.message.ServiceRequest(
        service="testservice",
        args=pickle.dumps(("testarg",)),
        kwargs=pickle.dumps({'testkwarg': 'test'}),
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
    assert_equal(pickle.loads(parsedreq.args)[0], "testarg")
    assert_equal(pickle.loads(parsedreq.kwargs)["testkwarg"], "test")

    # assert parse returns the modified list
    parsedreq_oneline = zmp.message.ServiceRequest()._parse(req.serialize())
    assert_true(isinstance(parsedreq_oneline, zmp.message.ServiceRequest))
    assert_true(parsedreq_oneline.initialized())
    assert_equal(parsedreq_oneline.service, "testservice")
    assert_equal(pickle.loads(parsedreq_oneline.args)[0], "testarg")
    assert_equal(pickle.loads(parsedreq_oneline.kwargs)["testkwarg"], "test")

    # run actual tuple-compatible online test
    oneline_symmetric_serialize_parse_servicerequest(req)


def test_symmetric_serialize_parse_serviceresponse_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceResponse(
        service="testservice",
        response=pickle.dumps("testresponse")
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
    assert_equal(parsedresp.service, "testservice")
    assert_equal(pickle.loads(parsedresp.response), "testresponse")

    # assert parse returns the modified list
    parsedresp_oneline = zmp.message.ServiceResponse()._parse(resp.serialize())
    assert_true(isinstance(parsedresp_oneline, zmp.message.ServiceResponse))
    assert_true(parsedresp_oneline.initialized())
    assert_equal(parsedresp_oneline.service, "testservice")
    assert_equal(pickle.loads(parsedresp_oneline.response), "testresponse")

    # run actual tuple-compatible online test
    oneline_symmetric_serialize_parse_serviceresponse(resp)


def test_symmetric_serialize_parse_serviceexception_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceException(
        exc_type="testexception",
        exc_value=pickle.dumps("testexceptionvalue"),
        traceback=pickle.dumps("testtraceback"),
    )
    # Check we have desired implementation
    assert_true(isinstance(resp, zmp.message.ServiceExceptionImpl))
    # Check it is an instance of Dynamic Functional Facade
    assert_true(isinstance(resp, zmp.message.ServiceException))

    assert_true(resp.initialized())

    # assert parse modifies the current message
    parsedresp = zmp.message.ServiceException()
    parsedresp._parse(resp.serialize())
    assert_true(isinstance(parsedresp, zmp.message.ServiceException))
    assert_true(parsedresp.initialized())
    assert_equal(parsedresp.exc_type, "testexception")
    assert_equal(pickle.loads(parsedresp.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(parsedresp.traceback), "testtraceback")

    # assert parse returns the modified list
    parsedresp_oneline = zmp.message.ServiceException()._parse(resp.serialize())
    assert_true(isinstance(parsedresp_oneline, zmp.message.ServiceException))
    assert_true(parsedresp_oneline.initialized())
    assert_equal(parsedresp_oneline.exc_type, "testexception")
    assert_equal(pickle.loads(parsedresp_oneline.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(parsedresp_oneline.traceback), "testtraceback")

    # run actual tuple-compatible online test
    oneline_symmetric_serialize_parse_serviceexception(resp)


def test_symmetric_serialize_parse_serviceresponseexception_protobuf():
    #Test Initialization
    resp = zmp.message.ServiceResponse(
        service="testservice",
        exception=zmp.message.ServiceException(
            exc_type="testexception",
            exc_value=pickle.dumps("testexceptionvalue"),
            traceback=pickle.dumps("testtraceback"),
        )
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
    assert_equal(parsedresp.service, "testservice")
    assert_equal(parsedresp.HasField('response'), False)
    assert_equal(parsedresp.HasField('exception'), True)
    assert_equal(parsedresp.exception.exc_type, "testexception")
    assert_equal(pickle.loads(parsedresp.exception.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(parsedresp.exception.traceback), "testtraceback")

    # assert parse returns the modified list
    parsedresp_oneline = zmp.message.ServiceResponse()._parse(resp.serialize())
    assert_true(isinstance(parsedresp_oneline, zmp.message.ServiceResponse))
    assert_true(parsedresp_oneline.initialized())
    assert_equal(parsedresp_oneline.service, "testservice")
    assert_equal(parsedresp_oneline.HasField('response'), False)
    assert_equal(parsedresp_oneline.HasField('exception'), True)
    assert_equal(parsedresp_oneline.exception.exc_type, "testexception")
    assert_equal(pickle.loads(parsedresp_oneline.exception.exc_value), "testexceptionvalue")
    assert_equal(pickle.loads(parsedresp_oneline.exception.traceback), "testtraceback")

    # run actual tuple-compatible online test
    oneline_symmetric_serialize_parse_serviceresponseexception(resp)


if __name__ == '__main__':

    import nose
    nose.runmodule()
