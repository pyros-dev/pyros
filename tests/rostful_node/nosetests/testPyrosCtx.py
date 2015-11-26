from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

from rostful_node.pyros_ctx import pyros_ctx
from rostful_node.mockinterface import PyrosMock
from rostful_node.pyros_client import PyrosClient

def testPyrosCtx():
    with pyros_ctx(mock=True) as ctx:
        assert isinstance(ctx.client, PyrosClient)

    # TODO : assert the context manager does his job ( HOW ? )

if __name__ == '__main__':

    import nose
    nose.runmodule()
