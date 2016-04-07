from __future__ import absolute_import

from collections import deque, namedtuple

from ..baseinterface import BaseService

ServiceType = namedtuple("ServiceType", "reqtype resptype")

statusecho_service = ServiceType("StatusMsg", "StatusMsg")


class MockService(BaseService):
    """
    MockService is a mock for the class handling conversion from python API to Service call
    """
    def __init__(self, service_name, service_type):
        super(MockService, self).__init__(service_name, service_type)

    def cleanup(self):
        pass

    def call(self, rosreq=None):
        # simulating echo

        return rosreq

BaseService.register(MockService)
