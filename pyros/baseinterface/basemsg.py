from __future__ import absolute_import
import abc


class BaseMessage(object):

    """
    BaseMsg is a base class to handle message type conversion
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, content_dict):
        """
        This handles conversion between types, the same way that python converts types from one to another
        :param msg:
        :return:
        """
        pass

    def __repr__(self):
        """
        This returns a representation of the message
        :return:
        """
        pass

    def __dict__(self):
        """
        This returns a dict representation of the message.
        Should be the inverse of the constructor
        :return:
        """
        pass

