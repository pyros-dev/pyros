from __future__ import absolute_import

import abc

# TODO : move this out of interface, it should also be known by client since we transfer exceptions
# All Pyros Exception must be pickleable and have a message property
class PyrosException(Exception):
    __metaclass__ = abc.ABCMeta

    @abc.abstractproperty
    def message(self):
        return