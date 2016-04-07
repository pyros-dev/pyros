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

# TODO : this should probably in pyros, not in zmp...

try:
    from .service_pb2 import ServiceRequestImpl, ServiceResponseImpl, ServiceExceptionImpl
    _PROTOBUF = True
except (ImportError):
    logging.warn("ZMQ : Protobuf message implementation not found. Using pickle based protocol")
    # Note : pickleable interface is inferior since type is not checked on assignment.
    # however the functional pickle interface is better than the object oriented protobuf message interface.
    # Another difference is that namedtuple are readonly, which is okay in most cases.
    #
    # The pickleable interface might just be a useful fallback if you don't want to use protobuf.
    # In that case, just call force_namedtuple_implementation() after importing this module to override default setup.

protobuf_implementation_enabled = False

#TODO : check python3 to avoid too much hacking here
def force_protobuf_implementation():

    if not _PROTOBUF:
        logging.error("Cannot activate protobuf implementation. Protobuf package not imported.")
        return False

    global protobuf_implementation_enabled
    protobuf_implementation_enabled = True
    # Dynamic Functional Facade for internal message format

    global ServiceRequest, ServiceRequest_dictparse
    # Note : isinstance returns True for both class names
    ServiceRequest = ServiceRequestImpl
    ServiceRequest.initialized = ServiceRequestImpl.IsInitialized
    ServiceRequest.has_field = lambda s, f: s.HasField(f)
    ServiceRequest.serialize = ServiceRequestImpl.SerializeToString
    ServiceRequest._parse = lambda s, m: [s, s.ParseFromString(m)][0]  # returns itself after modification during parsing
    # Factory Function for oneline parsing creation
    ServiceRequest_dictparse = lambda m: ServiceRequest(**{f.name: v for (f, v) in ServiceRequest()._parse(m).ListFields()})

    global ServiceResponse, ServiceResponse_dictparse
    # Note : isinstance returns True for both class names
    ServiceResponse = ServiceResponseImpl
    ServiceResponse.initialized = ServiceResponseImpl.IsInitialized
    ServiceResponse.has_field = lambda s, f: s.HasField(f)
    ServiceResponse.serialize = ServiceResponseImpl.SerializeToString
    ServiceResponse._parse = lambda s, m: [s, s.ParseFromString(m)][0]  # returns itself after modification during parsing
    # Factory Function for oneline parsing creation
    ServiceResponse_dictparse = lambda m: ServiceResponse(**{f.name: v for (f, v) in ServiceResponse()._parse(m).ListFields()})
    #TODO : check if we can use CopyFrom() instead ?

    global ServiceException, ServiceException_dictparse
    # Note : isinstance returns True for both class names
    ServiceException = ServiceExceptionImpl
    ServiceException.initialized = ServiceExceptionImpl.IsInitialized
    ServiceException.has_field = lambda s, f: s.HasField(f)
    ServiceException.serialize = ServiceExceptionImpl.SerializeToString
    ServiceException._parse = lambda s, m: [s, s.ParseFromString(m)][0]  # returns itself after modification during parsing
    # Factory Function for oneline parsing creation
    ServiceException_dictparse = lambda m: ServiceException(**{f.name: v for (f, v) in ServiceException()._parse(m).ListFields()})

    # TODO : Generic / Template class / Functional Interface for any kind of message


# Fallback implementation. need to be defined at module level to be pickleable ( unless dill is used )
ServiceRequestNTImpl = namedtuple("ServiceRequest", "service args kwargs")
ServiceResponseNTImpl = namedtuple("ServiceResponse", "service response exception")
ServiceExceptionNTImpl = namedtuple("ServiceException", "exc_type exc_value traceback")


def force_namedtuple_implementation():
    global protobuf_implementation_enabled
    protobuf_implementation_enabled = False
    # Dynamic Functional Facade for internal message format

    global ServiceRequest, ServiceRequest_dictparse
    # Extend named tuple implementation
    ServiceRequest = ServiceRequestNTImpl
    ServiceRequest.__new__.__defaults__ = (None,) * len(ServiceRequest._fields)  # making all fields optional ( like in protobuf protocol )
    ServiceRequest.initialized = lambda s: True
    ServiceRequest.has_field = lambda s, f: getattr(s, f, None) is not None
    ServiceRequest.serialize = lambda s: pickle.dumps(s)
    # Factory Function for oneline parsing creation
    ServiceRequest_dictparse = lambda m: pickle.loads(m)

    global ServiceResponse, ServiceResponse_dictparse
    # Extend named tuple implementation
    ServiceResponse = ServiceResponseNTImpl
    ServiceResponse.__new__.__defaults__ = (None,) * len(ServiceResponse._fields)  # making all fields optional ( like in protobuf protocol )
    ServiceResponse.initialized = lambda s: True
    ServiceResponse.has_field = lambda s, f: getattr(s, f, None) is not None
    ServiceResponse.serialize = lambda s: pickle.dumps(s)
    # Factory Function for oneline parsing creation
    ServiceResponse_dictparse = lambda m: pickle.loads(m)

    global ServiceException, ServiceException_dictparse
    # Extend named tuple implementation
    ServiceException = ServiceExceptionNTImpl
    ServiceException.__new__.__defaults__ = (None,) * len(ServiceException._fields)  # making all fields optional ( like in protobuf protocol )
    ServiceException.initialized = lambda s: True
    ServiceException.has_field = lambda s, f: getattr(s, f, None) is not None
    ServiceException.serialize = lambda s: pickle.dumps(s)
    # Factory Function for oneline parsing creation
    ServiceException_dictparse = lambda m: pickle.loads(m)

    # TODO : Generic / Template class / Functional Interface for any kind of message


# Default behavior : choose protobuf if available
if _PROTOBUF:
    force_protobuf_implementation()
else:
    force_namedtuple_implementation()


