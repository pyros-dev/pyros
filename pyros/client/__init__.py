# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function

# The client part should be the same for any kind of backend.
# The only requirement for this client is to be used in a python environment.
# Therefore it is not, and will not be a namespace package.

from .client import PyrosClient

__all__ = [
    'PyrosClient',
]




