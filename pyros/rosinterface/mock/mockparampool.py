from __future__ import absolute_import

from contextlib import contextmanager

from ...baseinterface import TransientIfPool
from .mocksystem import (
    params_available_remote, params_available_type_remote,
)

from .mockparam import MockParam


class MockParamPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, params=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        super(MockParamPool, self).__init__(params)

    # mock functions that simulate/mock similar interface than what is found on multiprocess framework supported
    # We should try our best to go for the lowest common denominator here
    # PARAMS
    def get_transients_available(self):  # function returning all params available on the system
        return self.available

    def transient_type_resolver(self, param_name):  # function resolving the type of a param
        prm = self.available.get(param_name)
        return prm  # None is returned if not found

    def TransientMaker(self, param_name, param_type, *args, **kwargs):  # the param class implementation
        return MockParam(param_name, param_type, *args, **kwargs)

    def TransientCleaner(self, param):  # the param class implementation
        return param.cleanup()

    def update(self):
        for p in params_available_remote:
            self.available[p] = params_available_type_remote.get(p)

        dt = self.transient_change_detect()

        return dt

TransientIfPool.register(MockParamPool)
