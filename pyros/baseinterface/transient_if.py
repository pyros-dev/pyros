from __future__ import absolute_import

class TransientIf(object):

    """
    TransientIf is a base class handling conversion from python API to Multiprocess concept.
    This concept can be a service, a param, a publisher, etc.
    This class aims at making this concept usable from python.
    It will be manipulated from TransientPool, to track when it goes up and down and when it should be interfaced.
    """

    def __init__(self, transient_name, transient_data=None):
        """
        Building a transient from external data
        :param transient_name:
        :param transient_data:
        """

        self.name = transient_name
        self.data = transient_data

    def cleanup(self):
        """
        cleanup function for a transient interface
        :return:
        """
        return

