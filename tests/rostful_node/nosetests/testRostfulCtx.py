from __future__ import absolute_import

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src')))

import roslaunch

from rostful_node.rostful_ctx import rostful_ctx
from rostful_node.rostful_mock import RostfulMock
from rostful_node.rostful_client import RostfulClient

def testRostfulCtx():
    with rostful_ctx(mock=True) as ctx:
        assert isinstance(ctx.client, RostfulClient)

    # TODO : assert the context manager does his job ( HOW ? )

if __name__ == '__main__':

    import nose
    nose.runmodule()
