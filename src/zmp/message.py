# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

import logging

# Protobuf is used if message has been generated.
# Otherwise use pickleable fallback
# Note : Many protobuf methods do not have implementation in pickle protocol.
_PROTOBUF = False

# Only for fallback behavior
import pickle
from collections import namedtuple

try:
    from .service_pb2 import ServiceRequestImpl, ServiceResponseImpl
    _PROTOBUF = True
except (ImportError):
    logging.warn("ZMQ : Protobuf message implementation not found. Using pickle based protocol")
    # Note : pickleable interface is inferior since type is not checked on assignment.
    # however the functional pickle interface is better than the object oriented protobuf message interface.
    # Another difference is that namedtuple are readonly, which is okay in most cases.
    #
    # The pickleable interface might just be a useful fallback if you don't want to use protobuf.
    # In that case, just call force_namedtuple_implementation() after importing this module to override default setup.

#TODO : check python3 to avoid too much hacking here
def force_protobuf_implementation():
    # Dynamic Functional Facade for internal message format

    global ServiceRequest, ServiceRequest_dictparse
    # Note : isinstance returns True for both class names
    ServiceRequest = ServiceRequestImpl
    ServiceRequest.initialized = ServiceRequestImpl.IsInitialized
    ServiceRequest.serialize = ServiceRequestImpl.SerializeToString
    ServiceRequest._parse = lambda s, m: [s, s.ParseFromString(m)][0]  # returns itself after modification during parsing
    # Factory Function for oneline parsing creation
    ServiceRequest_dictparse = lambda m: ServiceRequest(**{f.name: v for (f, v) in ServiceRequest()._parse(m).ListFields()})

    global ServiceResponse, ServiceResponse_dictparse
    # Note : isinstance returns True for both class names
    ServiceResponse = ServiceResponseImpl
    ServiceResponse.initialized = ServiceResponseImpl.IsInitialized
    ServiceResponse.serialize = ServiceResponseImpl.SerializeToString
    ServiceResponse._parse = lambda s, m: [s, s.ParseFromString(m)][0]  # returns itself after modification during parsing
    # Factory Function for oneline parsing creation
    ServiceResponse_dictparse = lambda m: ServiceResponse(**{f.name: v for (f, v) in ServiceResponse()._parse(m).ListFields()})
    #TODO : check if we can use CopyFrom() instead ?

    # TODO : Generic / Template class / Functional Interface for any kind of message


def force_namedtuple_implementation():
    # Dynamic Functional Facade for internal message format

    global ServiceRequest, ServiceRequestNTImpl, ServiceRequest_dictparse
    # Fallback implementation. need to be defined at module level to be pickleable ( unless dill is used )
    ServiceRequestNTImpl = namedtuple("ServiceRequest", "service args kwargs")
    # Extend named tuple implementation
    ServiceRequest = ServiceRequestNTImpl
    ServiceRequest.initialized = lambda s: True
    ServiceRequest.serialize = lambda s: pickle.dumps(s)
    # Factory Function for oneline parsing creation
    ServiceRequest_dictparse = lambda m: pickle.loads(m)

    global ServiceResponse, ServiceResponseNTImpl, ServiceResponse_dictparse
    # Fallback implementation. need to be defined at module level to be pickleable ( unless dill is used )
    ServiceResponseNTImpl = namedtuple("ServiceResponse", "type service response")
    # Extend named tuple implementation
    ServiceResponse = ServiceResponseNTImpl
    ServiceResponse.RESPONSE = 1  # CAREFUL this needs to match the protobuf definition
    ServiceResponse.ERROR = 2  # CAREFUL this needs to match the protobuf definition
    ServiceResponse.initialized = lambda s: True
    ServiceResponse.serialize = lambda s: pickle.dumps(s)
    # Factory Function for oneline parsing creation
    ServiceResponse_dictparse = lambda m: pickle.loads(m)

    # TODO : Generic / Template class / Functional Interface for any kind of message

# Default behavior : choose protobuf if available
if _PROTOBUF:
    force_protobuf_implementation()
else:
    force_namedtuple_implementation()


