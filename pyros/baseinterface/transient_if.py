from __future__ import absolute_import
from __future__ import print_function


class TransientIf(object):
    """
    TransientIf is a base class handling conversion from python API to Multiprocess transients.
    This transient can be a service, a param, a publisher, etc.
    This class aims at interfacing this transient with python in a common way for all multiprocess implementations.
    It will be manipulated from TransientPool, to track when this interface should be created or destroyed.
    """

    def __init__(self, transient_name, transient_type=None):
        """
        Building a transient from external data
        :param transient_name:
        :param transient_data:
        """

        self.name = transient_name
        self.type = transient_type

    def cleanup(self):
        """
        cleanup function for a transient interface
        :return:
        """
        return

