# -*- coding: utf-8 -*-
from __future__ import absolute_import

import warnings
import functools

# This is duplicated from pyros_setup here to not require pyros-setup as dependency when running from deb package.

# http://stackoverflow.com/questions/2536307/decorators-in-the-python-standard-lib-deprecated-specifically
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""

    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter('always', DeprecationWarning)  #turn off filter
        warnings.warn("Call to deprecated function {}.".format(func.__name__),
                      category=DeprecationWarning, stacklevel=2)
        warnings.simplefilter('default', DeprecationWarning)  #reset filter
        return func(*args, **kwargs)

    return new_func
