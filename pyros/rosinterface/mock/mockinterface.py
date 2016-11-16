from __future__ import absolute_import

from .mockservicepool import MockServicePool
from .mocksubscriberpool import MockSubscriberPool
from .mockpublisherpool import MockPublisherPool
from .mockparampool import MockParamPool

from ..baseinterface import BaseInterface


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
