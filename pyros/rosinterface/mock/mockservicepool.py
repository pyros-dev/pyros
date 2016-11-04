from __future__ import absolute_import

from contextlib import contextmanager

from ...baseinterface import TransientIfPool
from .mocksystem import (
    services_available_remote, services_available_type_remote,
)


from .mockservice import MockService


class MockServicePool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, services=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        super(MockServicePool, self).__init__(services)

    # mock functions that simulate/mock similar interface than what is found on multiprocess framework supported
    # We should try our best to go for the lowest common denominator here
    # SERVICES
    def get_transients_available(self):  # function returning all services available on the system
        return self.available

    def transient_type_resolver(self, service_name):  # function resolving the type of a service
        svc = self.available.get(service_name)
        return svc  # None is returned if not found

    def TransientMaker(self, service_name, service_type, *args, **kwargs):  # the service class implementation
        return MockService(service_name, service_type, *args, **kwargs)

    def TransientCleaner(self, service):  # the service class cleanup implementation
        return service.cleanup()

    def update(self):
        for s in services_available_remote:
            self.available[s] = services_available_type_remote.get(s)

        dt = self.transient_change_detect()

        return dt

TransientIfPool.register(MockServicePool)

