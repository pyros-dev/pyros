# -*- coding: utf-8 -*-
# This python package is a set of base classes and utilities for all the possible multiprocess framework interfaces
# available in pyros.
# Using these should make it as simple as possible to implement correctly working interfaces
# between different multiprocess frameworks
#
from __future__ import absolute_import

from .transient_if import TransientIf
from .transient_if_pool import TransientIfPool, DiffTuple

__all__ = [
    'DiffTuple',
    'TransientIf',
    'TransientIfPool',
]
