# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import print_function



class UnknownServiceException(Exception):
    """
    Raised by a Service call if the destination node doesnt provide this service
    """
    pass


class UnknownRequestTypeException(Exception):
    """
    Raised by a Service call if the destination node doesnt recognize the request type
    """
    pass


class UnknownResponseTypeException(Exception):
    """
    Raised by a Service call if the service doesnt recognise the response type
    """
    pass
