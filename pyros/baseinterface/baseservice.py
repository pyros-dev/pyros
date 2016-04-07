from __future__ import absolute_import
import abc


class BaseService(object):

    """
    BaseService is a base class to handling conversion from python API to Service call
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, service_name, service_type):
        # service_type is a composition of python standard builtin types that still make sense outside of
        # the python environment where there originated from :
        # int, float, long, complex, str, unicode, list, tuple, bytearray, buffer, xrange, set, frozenset, dict
        self.name = service_name
        # setting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        self.srvtype = (service_type.reqtype, service_type.resptype)

    @abc.abstractmethod
    def cleanup(self):
        return

    @abc.abstractmethod
    def call(self, rosreq=None):
        return
