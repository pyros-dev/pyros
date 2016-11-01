from __future__ import absolute_import

from .mockservicepool import MockServicePool
from .mocktopicpool import MockTopicPool
from .mockparampool import MockParamPool


class MockInterface(object):

    """
    MockInterface.
    """
    def __init__(self, services=None, topics=None, params=None):
        self.service_pool = MockServicePool(services)
        self.topic_pool = MockTopicPool(topics)
        self.param_pool = MockParamPool(params)

    # REQUESTED
    @property
    def services_args(self):
        return self.service_pool.transients_args

    @property
    def topics_args(self):
        return self.topic_pool.transients_args

    @property
    def params_args(self):
        return self.param_pool.transients_args

    # INTERFACED
    @property
    def services(self):
        return self.service_pool.transients

    @property
    def topics(self):
        return self.topic_pool.transients

    @property
    def params(self):
        return self.param_pool.transients

    # EXPOSE
    def expose_services(self, svc_regex):
        return self.service_pool.expose_transients_regex(svc_regex)

    def expose_topics(self, tpc_regex):
        return self.topic_pool.expose_transients_regex(tpc_regex)

    def expose_params(self, prm_regex):
        return self.param_pool.expose_transients_regex(prm_regex)

    #MOCK
    def mock_service(self, svc_name, svc_type):
        return self.service_pool.mock(svc_name, svc_type)

    def mock_topic(self, tpc_name, tpc_type):
        return self.topic_pool.mock(tpc_name, tpc_type)

    def mock_param(self, prm_name, prm_type):
        return self.param_pool.mock(prm_name, prm_type)

    #CHANGE DIFF
    def services_change_diff(self, appeared, gone):
        return self.service_pool.transient_change_diff(appeared, gone)

    def topics_change_diff(self, appeared, gone):
        return self.topic_pool.transient_change_diff(appeared, gone)

    def params_change_diff(self, appeared, gone):
        return self.param_pool.transient_change_diff(appeared, gone)

    #CHAGNE DETECT
    def services_change_detect(self):
        return self.service_pool.transient_change_detect()

    def topics_change_detect(self):
        return self.topic_pool.transient_change_detect()

    def param_change_detect(self):
        return self.param_pool.transient_change_detect()

    #UPDATE
    def update_services(self, add_names, remove_names):
        return self.service_pool.update_transients(add_names, remove_names)

    def update_topics(self, add_names, remove_names):
        return self.topic_pool.update_transients(add_names, remove_names)

    def update_params(self, add_names, remove_names):
        return self.param_pool.update_transients(add_names, remove_names)


    def update(self):
        self.topic_pool.transient_change_detect()
        self.service_pool.transient_change_detect()
        self.param_pool.transient_change_detect()
