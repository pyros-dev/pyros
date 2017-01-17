from __future__ import absolute_import

from .mockservicepool import MockServicePool
from .mocksubscriberpool import MockSubscriberPool
from .mockpublisherpool import MockPublisherPool
from .mockparampool import MockParamPool


# TODO : merge this with mock interface... Goal is to split Mock and ROS interfaces...

# TODO Entity Component System design for interface loop. cf https://pypi.python.org/pypi/esper (py3 + py2 in fork)
# Entities are transients (ex : for ROS : pubs, subs, svcs, params, and more can be added),
# Systems store logic about when/how a transient should be represented in the interface.
# GOALS : clarity, testability and flexibility
class BaseInterface(object):

    """
    BaseInterface.
    Assumption : we only deal with absolute names here. The users should resolve them
    """

    # Abstract methods to override for topics ("pub/sub type" communication channel)
    def get_publisher_list(self):  # function returning all topics available on the system
        return self.publishers_if_pool.get_transients_available()

    def publisher_type_resolver(self, topic_name):  # function resolving the type of a topic
        """
        :param topic_name: the name of the topic
        :return: returns None if the topic cannot be found. Properly except in all other unexpected events.
        """
        return self.publishers_if_pool.transient_type_resolver(topic_name)

    def PublisherMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return self.publishers_if_pool.TransientMaker(topic_name, topic_type, *args, **kwargs)

    def PublisherCleaner(self, topic):  # the topic class implementation
        return self.publishers_if_pool.TransientCleaner(topic)

    # Abstract methods to override for topics ("pub/sub type" communication channel)
    def get_subscriber_list(self):  # function returning all topics available on the system
        return self.subscribers_if_pool.get_transients_available()

    def subscriber_type_resolver(self, topic_name):  # function resolving the type of a topic
        """
        :param topic_name: the name of the topic
        :return: returns None if the topic cannot be found. Properly except in all other unexpected events.
        """
        return self.subscribers_if_pool.transient_type_resolver(topic_name)

    def SubscriberMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return self.subscribers_if_pool.TransientMaker(topic_name, topic_type, *args, **kwargs)

    def SubscriberCleaner(self, topic):  # the topic class implementation
        return self.subscribers_if_pool.TransientCleaner(topic)

    def get_svc_list(self):  # function returning all services available on the system
        return self.services_if_pool.get_transients_available()

    def service_type_resolver(self, service_name):  # function resolving the type of a service
        """
        :param service_name: the name of the service
        :return: returns None if the type cannot be found. Properly except in all other unexpected events.
        """
        return self.services_if_pool.transient_type_resolver(service_name)

    def ServiceMaker(self, service_name, service_type, *args, **kwargs):  # the service class implementation
        return self.services_if_pool.TransientMaker(service_name, service_type, *args, **kwargs)

    def ServiceCleaner(self, service):  # the service class implementation
        return self.services_if_pool.TransientCleaner(service)

    # Abstract methods to override for topics ("pub/sub type" communication channel)
    def get_topic_list(self):  # function returning all topics available on the system
        return self.topics_pool.get_transients_available()

    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        """
        :param topic_name: the name of the topic
        :return: returns None if the topic cannot be found. Properly except in all other unexpected events.
        """
        return self.topics_pool.transient_type_resolver(topic_name)

    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return self.topics_pool.TransientMaker(topic_name, topic_type, *args, **kwargs)

    def TopicCleaner(self, topic):  # the topic class implementation
        return self.topics_pool.TransientCleaner(topic)

    # Abstract methods to override for params ( "global" get/set objects )
    def get_param_list(self):  # function returning all topics available on the system
        return self.params_if_pool.get_transients_available()

    def param_type_resolver(self, param_name):  # function resolving the type of a topic
        return self.params_if_pool.transient_type_resolver(param_name)

    def ParamMaker(self, param_name, param_type, *args, **kwargs):  # the param class implementation
        return self.params_if_pool.TransientMaker(param_name, param_type, *args, **kwargs)

    def ParamCleaner(self, param):  # the param class implementation
        return self.params_if_pool.TransientCleaner(param)

    def __init__(self, publishers_if_pool, subscribers_if_pool, services_if_pool, params_if_pool):
        """
        Initializes the interface instance, to expose services, topics, and params
        """
        # Current transients exposed, i.e. those which are
        # active in the system.

        self.params_if_pool = params_if_pool
        self.services_if_pool = services_if_pool
        self.subscribers_if_pool = subscribers_if_pool
        self.publishers_if_pool = publishers_if_pool

    # REQUESTED
    @property
    def publishers_args(self):
        return self.publishers_if_pool.transients_args

    @property
    def subscribers_args(self):
        return self.subscribers_if_pool.transients_args

    @property
    def services_args(self):
        return self.services_if_pool.transients_args

    @property
    def params_args(self):
        return self.params_if_pool.transients_args

    # AVAILABLE
    @property
    def publishers_available(self):
        return self.publishers_if_pool.available

    @property
    def subscribers_available(self):
        return self.subscribers_if_pool.available

    @property
    def services_available(self):
        return self.services_if_pool.available

    @property
    def params_available(self):
        return self.params_if_pool.available

    # INTERFACED
    @property
    def publishers(self):
        return self.publishers_if_pool.transients

    @property
    def subscribers(self):
        return self.subscribers_if_pool.transients

    @property
    def services(self):
        return self.services_if_pool.transients

    @property
    def params(self):
        return self.params_if_pool.transients

    # EXPOSE
    def expose_publishers(self, tpc_regex):
        return self.publishers_if_pool.expose_transients_regex(tpc_regex)

    def expose_subscribers(self, tpc_regex):
        return self.subscribers_if_pool.expose_transients_regex(tpc_regex)

    def expose_services(self, svc_regex):
        return self.services_if_pool.expose_transients_regex(svc_regex)

    def expose_params(self, prm_regex):
        return self.params_if_pool.expose_transients_regex(prm_regex)

    #CHANGE DIFF
    def publishers_change_diff(self, appeared, gone):
        return self.publishers_if_pool.transient_change_diff(appeared, gone)

    def subscribers_change_diff(self, appeared, gone):
        return self.subscribers_if_pool.transient_change_diff(appeared, gone)

    def services_change_diff(self, appeared, gone):
        return self.services_if_pool.transient_change_diff(appeared, gone)

    def params_change_diff(self, appeared, gone):
        return self.params_if_pool.transient_change_diff(appeared, gone)

    #CHANGE DETECT
    def publishers_change_detect(self):
        return self.publishers_if_pool.transient_change_detect()

    def subscribers_change_detect(self):
        return self.subscribers_if_pool.transient_change_detect()

    def services_change_detect(self):
        return self.services_if_pool.transient_change_detect()

    def param_change_detect(self):
        return self.params_if_pool.transient_change_detect()

    #UPDATE
    def update_publishers(self, add_names, remove_names):
        return self.publishers_if_pool.update_transients(add_names, remove_names)

    def update_subscribers(self, add_names, remove_names):
        return self.subscribers_if_pool.update_transients(add_names, remove_names)

    def update_services(self, add_names, remove_names):
        return self.services_if_pool.update_transients(add_names, remove_names)

    def update_params(self, add_names, remove_names):
        return self.params_if_pool.update_transients(add_names, remove_names)

    # TODO : "wait_for_it" methods that waits for hte detection of a topic/service on the system
    # TODO : Should return a future so use can decide to wait on it or not
    # TODO : Maybe similar to a async_detect ( hooked up to the detected transient, not the exposed ones )
    # TODO : Exposed interface is for direct control flow => async not really needed
    # TODO : Detect/Update interface is inversed control flow ( from update loop ) => Needed



class MockInterface(BaseInterface):

    """
    MockInterface. A simple mock following ROS design.
    Userful to test the multiprocess parts of pyros without worrying about real connexion with ROS.
    """
    def __init__(self, publishers=None, subscribers=None, services=None, params=None):

        publishers_pool = MockPublisherPool(publishers)
        subscribers_pool = MockSubscriberPool(subscribers)
        services_pool = MockServicePool(services)
        params_pool = MockParamPool(params)

        super(MockInterface, self).__init__(publishers_pool, subscribers_pool, services_pool, params_pool)

    #
    # # REQUESTED
    # @property
    # def services_args(self):
    #     return self.service_pool.transients_args
    #
    # @property
    # def topics_args(self):
    #     return self.topic_pool.transients_args
    #
    # @property
    # def params_args(self):
    #     return self.param_pool.transients_args
    #
    # # AVAILABLE
    # @property
    # def services_available(self):
    #     return self.service_pool.available
    #
    # @property
    # def topics_available(self):
    #     return self.topic_pool.available
    #
    # @property
    # def params_available(self):
    #     return self.param_pool.available
    #
    # # INTERFACED
    # @property
    # def services(self):
    #     return self.service_pool.transients
    #
    # @property
    # def topics(self):
    #     return self.topic_pool.transients
    #
    # @property
    # def params(self):
    #     return self.param_pool.transients
    #
    # # EXPOSE
    # def expose_services(self, svc_regex):
    #     return self.service_pool.expose_transients_regex(svc_regex)
    #
    # def expose_topics(self, tpc_regex):
    #     return self.topic_pool.expose_transients_regex(tpc_regex)
    #
    # def expose_params(self, prm_regex):
    #     return self.param_pool.expose_transients_regex(prm_regex)
    #
    #MOCK
    def mock_publisher(self, tpc_name, tpc_type):
        return self.publishers_if_pool.mock(tpc_name, tpc_type)

    def mock_subscriber(self, tpc_name, tpc_type):
        return self.subscribers_if_pool.mock(tpc_name, tpc_type)

    def mock_service(self, svc_name, svc_type):
        return self.services_if_pool.mock(svc_name, svc_type)

    def mock_param(self, prm_name, prm_type):
        return self.params_if_pool.mock(prm_name, prm_type)
    #
    # #CHANGE DIFF
    # def services_change_diff(self, appeared, gone):
    #     return self.service_pool.transient_change_diff(appeared, gone)
    #
    # def topics_change_diff(self, appeared, gone):
    #     return self.topic_pool.transient_change_diff(appeared, gone)
    #
    # def params_change_diff(self, appeared, gone):
    #     return self.param_pool.transient_change_diff(appeared, gone)
    #
    # #CHAGNE DETECT
    # def services_change_detect(self):
    #     return self.service_pool.transient_change_detect()
    #
    # def topics_change_detect(self):
    #     return self.topic_pool.transient_change_detect()
    #
    # def param_change_detect(self):
    #     return self.param_pool.transient_change_detect()
    #
    # #UPDATE
    # def update_services(self, add_names, remove_names):
    #     return self.service_pool.update_transients(add_names, remove_names)
    #
    # def update_topics(self, add_names, remove_names):
    #     return self.topic_pool.update_transients(add_names, remove_names)
    #
    # def update_params(self, add_names, remove_names):
    #     return self.param_pool.update_transients(add_names, remove_names)

    def update(self):
        self.publishers_if_pool.update()
        self.subscribers_if_pool.update()
        self.services_if_pool.update()
        self.params_if_pool.update()
