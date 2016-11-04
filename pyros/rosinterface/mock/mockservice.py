from __future__ import absolute_import

from collections import deque, namedtuple

from ...baseinterface import TransientIf

ServiceType = namedtuple("ServiceType", "reqtype resptype")

statusecho_service = ServiceType("StatusMsg", "StatusMsg")


class MockService(TransientIf):
    """
    MockService is a mock for the class handling conversion from python API to Service call
    """
    def __init__(self, service_name, service_type):
        # setting the name to make sure we start with /
        service_name = service_name if service_name.startswith('/') else '/' + service_name

        # service_type is a composition of python standard builtin types that still make sense outside of
        # the python environment where there originated from :
        # int, float, long, complex, str, unicode, list, tuple, bytearray, buffer, xrange, set, frozenset, dict
        service_type = (service_type.reqtype, service_type.resptype)

        # Initializing the transient
        super(MockService, self).__init__(service_name, service_type)

    def cleanup(self):
        pass

    def call(self, rosreq=None):
        # simulating echo

        return rosreq

