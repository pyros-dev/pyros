from __future__ import absolute_import

from pyros.client.client import PyrosClient
from pyros.ctx_server import pyros_ctx


def testPyrosMockCtx():
    with pyros_ctx(mock_node=True) as ctx:
        assert isinstance(ctx.client, PyrosClient)

    # TODO : assert the context manager does his job ( HOW ? )


if __name__ == '__main__':

    import nose
    nose.runmodule()
