# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function

from collections import namedtuple

# Internal message format - alternative to protobuf but using similar interface for convenience
ServiceRequestImpl = namedtuple("ServiceRequest", "service request")
ServiceResponseImpl = namedtuple("ServiceResponse", "type service response")
# adding enums ( similar to protobuf interface )
ServiceResponse.RESPONSE = 1
ServiceResponse.ERROR = 2
