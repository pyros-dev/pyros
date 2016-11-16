from __future__ import absolute_import

from .rosinterface.mock import MockInterface
from . import config
from .basenode import PyrosBase


class PyrosMock(PyrosBase):
    """
    Mock Interface in pure python ( No ROS needed ).

    """
    #: Default configuration parameters.
    _default_config = {
        'TOPICS': [],
        'SERVICES': [],
        'PARAMS': [],
    }

    # TODO : we probably want to reuse the standard mock module here...
    def __init__(self, name=None, pyros_config=None, args=None, kwargs=None):
        name = name or 'pyros-mock'

        # No default config here for now

        # TODO : use context manager here to use another process just like any resource...
        super(PyrosMock, self).__init__(
            name,
            interface_class=MockInterface,
            args=args or (),
            kwargs=kwargs or {},
            default_config=self._default_config,  # we pass our default config here
        )

        # overriding default config with parameter provided
        self.config_handler.configure(config)  # configuring with our package default
        if pyros_config:
            self.config_handler.configure(pyros_config)  # configuring with argument passed form user

        self._topic_msg = {}  # storage for the echo topic
        self._param_val = {}  # storage for the test param
        self._stop_event = None  # stop_event to signal the thread for soft shutdown
        self._spinner = None  # thread instance

    # These should match the design of PyrosClient and Protocol so we are consistent between pipe and python API
    def msg_build(self, name):
        # TODO : better mock : this can except if not string.
        msg = str()
        return msg

    # a simple echo topic
    def topic(self, name, msg_content=None):
        # TODO : use Mock interface topics directly
        msg = msg_content
        if msg_content is not None:
            self._topic_msg[name] = msg_content
            msg = None  # consuming the message
        else:
            msg = self._topic_msg.get(name)
        return msg
        
    def topics(self):
        """
        :return: the list of topics we interfaced with ( not the list of all available topics )
        """
        topics = self.interface.publishers.copy()
        topics.update(self.interface.subscribers)
        return topics

    # a simple publisher
    def subscriber(self, name, msg_content):
        # TODO : use Mock interface topics directly
        msg = msg_content
        self._topic_msg[name] = msg_content
        msg = None  # consuming the message
        return msg

    def subscribers(self):
        """
        :return: the list of topics we interfaced with ( not the list of all available topics )
        """
        return self.interface.subscribers

    def publishers(self):
        """
        :return: the list of topics we interfaced with ( not the list of all available topics )
        """
        return self.interface.publishers

    # a simple subscriber
    def publisher(self, name):
        # TODO : use Mock interface topics directly
        msg = self._topic_msg.get(name)
        return msg

    # a simple string echo service
    def service(self, name, rqst_content=None):
        # simulating a strict message typing backend
        #if not isinstance(rqst_content, str):
        #    raise Exception("Request Content Not Expected Type !")
        resp_content = rqst_content
        return resp_content

    def services(self):
        """
        :return: the list of services we interfaced with ( not the list of all available services )
        """
        return self.interface.services

    # a simple test param
    def param(self, name, value=None):
        val = value
        if value is not None:
            self._param_val[name] = value
            val = None  # consuming the message
        else:
            val = self._param_val.get(name)
        return val

    def params(self):
        """
        :return: the list of params we interfaced with ( not the list of all available params )
        """
        return self.interface.params

    def setup(self, publishers=None, subscribers=None, services=None, topics=None, params=None):
        """
        :param publishers:
        :param subscribers:
        :param services:
        :param topics: ONLY HERE for BW compat
        :param params:
        :return:
        """
        super(PyrosMock, self).setup(publishers=publishers, subscribers=subscribers, services=services, topics=topics, params=params)


PyrosBase.register(PyrosMock)
