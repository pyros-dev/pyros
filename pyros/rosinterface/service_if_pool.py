from __future__ import absolute_import

import logging
from itertools import ifilter

import rosservice

from .api import rospy_safe as rospy

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

from ..baseinterface import TransientIfPool

from .service_if import ServiceBack, ServiceTuple

try:
    import rocon_python_comms
except ImportError:
    rocon_python_comms = None


class RosServiceIfPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, services=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        super(RosServiceIfPool, self).__init__(services, transients_desc="services")

    def get_transients_available(self):  # function returning all services available on the system
        return self.available

    def transient_type_resolver(self, service_name):  # function resolving the type of a service
        svc = self.available.get(service_name)
        if svc:
            if svc.type is None:  # if the type is unknown, lets discover it (needed to interface)
                try:
                    resolved_service_name = rospy.resolve_name(service_name)  # required or not ?
                    svc.type = rosservice.get_service_type(resolved_service_name)
                except rosservice.ROSServiceIOException:  # exception can occur -> just reraise
                    raise
            return svc.type  # return the type
        else:
            rospy.logerr("ERROR while resolving {service_name}. Service not known as available. Ignoring".format(**locals()))
            return None

    def TransientMaker(self, service_name, service_type, *args, **kwargs):  # the service class implementation
        return ServiceBack(service_name, service_type, *args, **kwargs)

    def TransientCleaner(self, service):  # the service class cleanup implementation
        return service.cleanup()

    ## bwcompat
    # REQUESTED
    @property
    def services_args(self):
        return self.transients_args


    # AVAILABLE
    @property
    def services_available(self):
        return self.available

    # INTERFACED
    @property
    def services(self):
        return self.transients

    # EXPOSE
    def expose_services(self, svc_regex):
        return self.expose_transients_regex(svc_regex)

    def get_svc_available(self):  # function returning all services available on the system
        return self.get_transients_available

    def service_type_resolver(self, service_name):  # function resolving the type of a service
        return self.transient_type_resolver(service_name)


    def ServiceMaker(self, service_name, service_type):  # the service class implementation
        return self.TransientMaker(service_name, service_type)

    def ServiceCleaner(self, service):  # the service class cleanup implementation
        return self.TransientCleaner(service)
    ###########

    def reset_state(self, services, service_types):
        """
        Reset internal system state representation.
        expect lists in format similar to masterAPI.
        :param publishers:
        :param subscribers:
        :param services:
        :param topic_types:
        :param service_types:
        :return:
        """
        self.available = dict()
        for s in services:  # We assume s[1] is never empty here
            st = next(ifilter(lambda lst: s[0] == lst[0], service_types), [])
            stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
            self.available[stp.name] = stp

        # We still need to return DiffTuples
        return services

    def compute_state(self, services_dt, service_types_dt):
        """
        This is called only if there is a cache proxy with a callback, and expects DiffTuple filled up with names or types
        :param services_dt:
        :param publishers_dt:
        :param subscribers_dt:
        :return:
        """
        for s in services_dt.added:
            st = next(ifilter(lambda lst: s[0] == lst[0], service_types_dt.added), [])
            stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
            if stp.name in self.available:
                if self.available[stp.name].type is None and stp.type is not None:
                    self.available[stp.name].type = stp.type
            else:
                self.available[stp.name] = stp

        for s in services_dt.removed:
            st = next(ifilter(lambda lst: s[0] == lst[0], service_types_dt.removed), [])
            stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
            if stp.name in self.available:
                self.available.pop(stp.name, None)

        # We still need to return DiffTuples
        return services_dt

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update_delta(self, services_dt, service_types_dt=None):
        services_dt = self.compute_state(services_dt, service_types_dt or [])

        if services_dt.added or services_dt.removed:
            _logger.debug(
                rospy.get_name() + " Services Delta {services_dt}".format(**locals()))

        # update_services wants only names
        dt = self.transient_change_diff(
            transient_appeared=[s[0] for s in services_dt.added],
            transient_gone=[s[0] for s in services_dt.removed]
            # add_names=regexes_match_sublist(self.transients_args, [s[0] for s in services_dt.added]),
            # remove_names=[s[0] for s in services_dt.removed if s[0] not in self.get_transients_available()]
        )

        if dt.added or dt.removed:
            _logger.debug(
                rospy.get_name() + " Update Delta {dt}".format(**locals()))
        return dt


    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    # @profile
    def update(self, services, service_types):
        # First we need to reflect the external system state in internal cache
        self.reset_state(services, service_types)

        if services:
            _logger.debug(
                rospy.get_name() + " Services List {services}".format(**locals()))

        # Second we update our interfaces based on that new system state
        # TODO : pass full services state here to avoid having to retrieve indirectly
        dt = self.transient_change_detect()

        if dt.added or dt.removed:
            _logger.debug(
                rospy.get_name() + " Update Delta {dt}".format(**locals()))

        return dt


TransientIfPool.register(RosServiceIfPool)

