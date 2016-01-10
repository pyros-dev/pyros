#This python package is handling all ROCON related communication for rostful-node.
from __future__ import absolute_import

import logging

try:
    from .interaction_watcher import InteractionWatcher
    __all__ = ['InteractionWatcher']
except ImportError as exc:
    logging.warn("import failed in {name}. disabling functionality".format(name=__name__))
    __all__ = []
