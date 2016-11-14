from __future__ import absolute_import
from __future__ import print_function

import logging

from .api import rospy_safe as rospy

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros should configure handlers

from .param_if import ParamBack, ParamTuple

from ..baseinterface import TransientIfPool, DiffTuple


class RosParamIfPool(TransientIfPool):

    """
    MockInterface.
    """
    def __init__(self, params=None):
        # This base constructor assumes the system to interface with is already available ( can do a get_svc_available() )
        super(RosParamIfPool, self).__init__(params, transients_desc="parameters")

    # mock functions that simulate/mock similar interface than what is found on multiprocess framework supported
    # We should try our best to go for the lowest common denominator here
    # PARAMS
    def get_transients_available(self):  # function returning all params available on the system
        return self.available

    def transient_type_resolver(self, param_name):  # function resolving the type of a param
        prm = self.available.get(param_name)
        if prm:
            if prm.type is None:  # if the type is unknown, lets discover it (since the param is supposed to exist)
                try:
                    prm.type = type(rospy.get_param(param_name))  # we use the detected python type here (since there is no rospy param type interface for this)
                except KeyError:  # exception can occur -> just reraise
                    raise
            return prm.type  # return the first we find. enough.
        else:
            rospy.logerr("ERROR while resolving {param_name}. Param not known as available. Ignoring".format(**locals()))
            return None

    def TransientMaker(self, param_name, param_type, *args, **kwargs):  # the param class implementation
        return ParamBack(param_name, param_type, *args, **kwargs)

    def TransientCleaner(self, param):  # the param class implementation
        return param.cleanup()

    ##bwcompat
    # REQUESTED
    @property
    def params_args(self):
        return self.transients_args

    # AVAILABLE
    @property
    def params_available(self):
        return self.available

    # INTERFACED
    @property
    def params(self):
        return self.transients

    # EXPOSE
    def expose_params(self, prm_regex):
        return self.expose_transients_regex(prm_regex)

    def get_param_available(self):  # function returning all params available on the system
        return self.get_transients_available()

    def param_type_resolver(self, param_name):  # function resolving the type of a param
        prm = self.transient_type_resolver(param_name)

    def ParamMaker(self, param_name, param_type):  # the param class implementation
        return self.TransientMaker(param_name, param_type)

    def ParamCleaner(self, param):  # the param class implementation
        return self.TransientCleaner(param)
    ##############3

    def retrieve_state(self):
        # TODO : maybe not here ? (to match services and topic behavior... and unify the request)
        """
        called to update params from rospy.
        CAREFUL : this can be called from another thread (subscriber callback)
        """
        params = rospy.get_param_names()
        self.reset_state(params)

    def reset_state(self, params):
        """
        called to update params from rospy.
        CAREFUL : this can be called from another thread (subscriber callback)
        """
        self.available = dict()
        for p in params:
            pt = []
            ptp = ParamTuple(name=p, type=pt[1] if len(pt) > 0 else None)
            self.available[ptp.name] = ptp

    def compute_state(self, params_dt):
        """
        called to update params from rospy.
        CAREFUL : this can be called from another thread (subscriber callback)
        """

        computed_params_dt = DiffTuple([], [])
        for p in params_dt.added:
            pt = ParamTuple(name=p, type=None)
            if pt.name in self.available:
                if self.available[pt.name].type is None or pt.type is not None:
                    self.available[pt.name].type = pt.type
            else:
                self.available[pt.name] = pt
                computed_params_dt.added.append(pt.name)

        for p in params_dt.removed:
            pt = ParamTuple(name=p, type=None)
            if pt.name in self.available:
                self.available.pop(pt.name, None)
                computed_params_dt.removed.append(pt.name)

        return computed_params_dt


    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    #@profile
    def update_delta(self, params_dt):

        # First we need to reflect the external system state in internal cache
        computed_params_dt = self.compute_state(params_dt)

        if computed_params_dt.added or computed_params_dt.removed:
            _logger.debug(
                rospy.get_name() + " Params Delta {params_dt}".format(**locals()))

        # _logger.debug("Params ADDED : {0}".format([p for p in available_dt.added]))
        # _logger.debug("Params GONE : {0}".format([p for p in available_dt.removed]))

        # Second we update our interfaces based on that system state difference
        dt = self.transient_change_diff(
            transient_appeared=computed_params_dt.added,
            transient_gone=computed_params_dt.removed
        )

        if dt.added or dt.removed:
            _logger.debug(rospy.get_name() + " Update Delta {dt}".format(**locals()))
        return dt

    # for use with line_profiler or memory_profiler
    # Not working yet... need to solve multiprocess profiling issues...
    #@profile
    def update(self, params):

        # First we need to reflect the external system state in internal cache
        self.reset_state(params)

        if params:
            _logger.debug(
                rospy.get_name() + " Params List {params}".format(**locals()))

        # Second we update our interfaces based on that new system state
        # TODO : pass full params state here to avoid having to retrieve indirectly
        dt = self.transient_change_detect()

        if dt.added or dt.removed:
            _logger.debug(rospy.get_name() + " Update Delta {dt}".format(**locals()))
        return dt


TransientIfPool.register(RosParamIfPool)
