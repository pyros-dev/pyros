from __future__ import absolute_import

import logging
import six
import sys
import threading
import collections
import re
import abc
from functools import partial

# module wide to be pickleable
from contextlib import contextmanager

DiffTuple = collections.namedtuple("DiffTuple", " added removed ")

from .regex_tools import regex_match_sublist, regexes_match_sublist, find_first_regex_match


#TODO Entity Component System design for interface loop. cf https://pypi.python.org/pypi/esper (py3 + py2 in fork)
# Entities are transients (ex : for ROS : pubs, subs, svcs, params, and more can be added),
# Systems store logic about when/how a transient should be represented in the interface.
# GOALS : clarity, testability and flexibility
class TransientIfPool(object):
    # TODO : split available into TransientPool
    # TODO : have update and update_delta managed here in generic way...
    """

    The TransientIfPool class maintains :

    - Available transients collection:
    {
      transient1_name: transient1_class_instance,
      transient2_name: transient2_class_instance,
      transient3_name: transient3_class_instance,
    }

    - Interfaced transients collection:
    {
      transient1_name: transient_if1_class_instance,
      transient3_name: transient_if3_class_instance
    }

    update() checks (or get fed) the recent system state (or diff), and the interface store internally what transients are available.
    Also the interface creates/destroy the transients interface class instance when needed.

    Assumption : we only deal with absolute names here. The users should resolve them
    """
    __metaclass__ = abc.ABCMeta

    def update_transients(self, add_names, remove_names, *class_build_args, **class_build_kwargs):
        """
        Adds transients (service, topic, etc) named in add_names if they are not exposed in resolved_dict
        and removes transients named in remove_names if they are exposed in resolved_dict

        This method can be used to force exposing/withholding any transient, bypassing other mechanisms.
        No extra check is made here regarding regex or system status to try to guess if it should be added/removed.
        This is left ot the caller.

        :param add_names: the names of the transients to add
        :param remove_names: the names of the transients to remove
        :param class_build_args: the args to pass to the resolved transient constructor
        :param class_build_kwargs: the kwargs to pass to the resolved transient constructor
        :return: the list of transients exposed
        """
        # Important : no effect if names is empty list. only return empty. functional style.

        added = []
        removed = []
        for tst_name in [tst for tst in add_names if tst not in self.transients.keys()]:
            try:
                ttype = self.transient_type_resolver(tst_name)  # should return None if error - TODO : handle (and reraise) exception !!
                if ttype is not None:  # transient can be resolved
                    self.transients[tst_name] = self.TransientMaker(tst_name, ttype, *class_build_args, **class_build_kwargs)
                    added += [tst_name]
                    logging.info("[{name}] Interfacing with {desc} {transient}".format(name=__name__, desc=self.transients_desc, transient=tst_name))
                else:
                    logging.warning("[{name}] Type of {desc} {transient} unknown. Giving up trying to interface.".format(name=__name__, desc=self.transients_desc,
                                                                              transient=tst_name))
            except Exception as e:
                logging.warn("[{name}] Cannot interface with {desc} {transient} : {exc}".format(name=__name__, desc=self.transients_desc, transient=tst_name, exc=e))
                exc_info = sys.exc_info()
                six.reraise(exc_info[0], exc_info[1], exc_info[2])

        for tst_name in [tst for tst in remove_names if tst in self.transients.keys()]:
            if tst_name in self.transients:  # we make sure the transient is still exposed
                # because we might have modified resolved_dict after building the list to loop on
                logging.info("[{name}] Removing {desc} {transient}".format(name=__name__, desc=self.transients_desc, transient=tst_name))
                self.TransientCleaner(self.transients[tst_name])  # calling the cleanup function in case we need to do something
                self.transients.pop(tst_name, None)
                removed += [tst_name]

        return DiffTuple(added, removed)

    def expose_transients_regex(self, regexes, *class_build_args, **class_build_kwargs):
        """
        Exposes a list of transients regexes. resolved transients not matching the regexes will be removed.
        _expose_transients_regex -> _transient_change_detect -> _transient_change_diff -> _update_transients
        :param regexes: the list of regex to filter the transient to add.
               Note: regexes = [] remove all registered regexes.
        :param regex_set: the list of regex already existing
        :return: the list of transients exposed
        """
        # Important : no effect if names is empty list. only return empty diff. functional style.

        # look through the new service names received by reconfigure, and add
        # those services which are not in the existing service args
        for tst_regex in [r for r in regexes if not r in self.transients_args]:
            self.transients_args.add(tst_regex)
            logging.info('[{name}] Exposing {desc} regex : {regex}'.format(name=__name__, desc=self.transients_desc, regex=tst_regex))
            # TODO : check here for bugs & add test : what if we add multiple regexes ? wont we miss some add_names ?

        # look through the current service args and delete those values which
        # will not be valid when the args are replaced with the new ones. run on
        # a copy so that we will remove from the original without crashing
        for tst_regex in [r for r in self.transients_args if not r in regexes]:
            logging.info('[{name}] Withholding {desc} regex : {regex}'.format(name=__name__, desc=self.transients_desc, regex=tst_regex))
            self.transients_args.remove(tst_regex)

        return self.transient_change_detect(
            *class_build_args,
            **class_build_kwargs
         )

    # TODO: pass hte status as arguments instead of using self.get_transient_avialable
    def transient_change_detect(self, *class_build_args, **class_build_kwargs):
        """
        This should be called when we want to detect a change in the status of the system regarding the transient list
        This function also applies changes due to regex_set updates if needed
        _transient_change_detect -> _transient_change_diff -> _update_transients
        """

        transient_detected = set(self.get_transients_available())
        #TODO : unify that last_got_set with the *_available. they are essentially the same
        tst_gone = self.last_transients_detected - transient_detected

        # print("INTERFACING + {transient_detected}".format(**locals()))
        # print("INTERFACING - {tst_gone}".format(**locals()))

        dt = self.transient_change_diff(
            # we start interfacing with new matches,
            # but we also need to update old matches that match regex now
            transient_appeared=transient_detected,
            transient_gone=tst_gone,
            *class_build_args,
            **class_build_kwargs
        )

        self.last_transients_detected.update(transient_detected)
        if tst_gone:
            self.last_transients_detected.difference_update(tst_gone)

        return dt

    def transient_change_diff(self, transient_appeared, transient_gone, *class_build_args, **class_build_kwargs):
        """
        This should be called when we want to process a change in the status of the system (if we already have the diff)
        This function also applies changes due to regex_set updates if needed
        _transient_change_diff -> _update_transients
        """

        to_add = set(regexes_match_sublist(self.transients_args, transient_appeared))
        lost_matches = {n for n in self.transients if find_first_regex_match(n, self.transients_args) is None}
        to_remove = set(transient_gone) | lost_matches  # we stop interfacing with lost transient OR lost matches

        return self.update_transients(
            add_names=to_add,
            remove_names=to_remove,
            *class_build_args,
            **class_build_kwargs
        )

    # Abstract methods to override for services ("RPC / request / call type" communication channel)
    @abc.abstractmethod
    def get_transients_available(self):  # function returning all services available on the system
        return

    @abc.abstractmethod
    def transient_type_resolver(self, transient_name):  # function resolving the type of a service
        """
        :param service_name: the name of the service
        :return: returns None if the type cannot be found. Properly except in all other unexpected events.
        """
        return

    @abc.abstractmethod
    def TransientMaker(self, transient_name, service_type, *args, **kwargs):  # the service class implementation
        return

    @abc.abstractmethod
    def TransientCleaner(self, transient):  # the transient class implementation
        return

    def __init__(self, transients=None, transients_desc=None):
        """
        Initializes the interface instance, to expose services, topics, and params
        """

        # This is a local cache of the system state because we don't want to ask everytime.
        # TODO: find a way to make interface development easier by allowing developer to compare and worry only about local state representation versus interface
        # NOT about how to synchronize remote state with local state...
        self.available = dict()
        # This stays always in sync with the system (via interface update call)
        # but the interface instance can be created and recreated independently

        # Current transients exposed, i.e. those which are
        # active in the system.
        self.transients = {}
        self.transients_desc = transients_desc

        # Last requested transients to be exposed, received
        # from a setup request.
        self.transients_args = set()

        # to allow comparison with previously detected services / topics/ params
        # TODO : get rid of this
        self.last_transients_detected = set()

        # First update() will re-detect previously found names, but when trying to update, no duplicates should happen.
        # This should be taken care of in the abstract transient functional interface

        if transients is not None:
            tdt = self.expose_transients_regex(transients)
        else:
            tdt = DiffTuple([], [])  # no change in exposed services

    # TODO : implement this to abstract from lower logic
    # TODO : split this into a separate transient_pool
    # def reset_state(self, transients, transients_type):
    #     """
    #     Reset internal system state representation.
    #     expect lists in format similar to masterAPI. <= TODO : change this so data transform is done once only, outside of generic code
    #     :param transients:
    #     :param transients_data:
    #     :return:
    #     """
    #     self.available = dict()
    #     for s in services:  # We assume s[1] is never empty here
    #         st = next(ifilter(lambda lst: s[0] == lst[0], service_types), [])
    #         stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
    #         self.available[stp.name] = stp
    #
    #     # We still need to return DiffTuples
    #     return services
    #
    # def compute_state(self, transients_added, transients_removed, service_types_dt):
    #     """
    #     This is called only if there is a cache proxy with a callback, and expects DiffTuple filled up with names or types
    #     :param services_dt:
    #     :param publishers_dt:
    #     :param subscribers_dt:
    #     :return:
    #     """
    #     for s in services_dt.added:
    #         st = next(ifilter(lambda lst: s[0] == lst[0], service_types_dt.added), [])
    #         stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
    #         if stp.name in self.available:
    #             if self.available[stp.name].type is None and stp.type is not None:
    #                 self.available[stp.name].type = stp.type
    #         else:
    #             self.available[stp.name] = stp
    #
    #     for s in services_dt.removed:
    #         st = next(ifilter(lambda lst: s[0] == lst[0], service_types_dt.removed), [])
    #         stp = ServiceTuple(name=s[0], type=st[1] if len(st) > 0 else None)
    #         if stp.name in self.available:
    #             self.available.pop(stp.name, None)
    #
    #     # We still need to return DiffTuples
    #     return services_dt


    @contextmanager
    def mock(self, tst_name, tst_type):
        """
        Mock appearance/disappearance to force changes in interface , via setting local cache.
        :param tst_name:
        :param tst_type:
        :return:
        """
        print(" -> Mock {tst_name} appear".format(**locals()))
        # Service appears
        self.available[tst_name] = tst_type
        yield
        # Service disappear
        self.available.pop(tst_name)
        print(" -> Mock {tst_name} disappear".format(**locals()))

    # def update_on_diff(self, transients_dt):
    #
    #     # For added transients we need to check they match the regex, otherwise we skip it.
    #     # For removed transients we need to check that they have disappeared from system before removing, otherwise we skip it.
    #     # TODO : investigate shutdown behavior more in details
    #     # if shutting_down:
    #     #     # We force removing all interfaces by calling directly update services
    #     #     sdt = self.update_services(add_names=[], remove_names=[s for s in self.services])
    #     #     tdt = self.update_topics(add_names=[], remove_names=[t for t in self.topics])
    #     #     pdt = self.update_params(add_names=[], remove_names=[p for p in self.params])
    #     #else:
    #     tdt = self.update_services(add_names=regexes_match_sublist(self.transients_args, transients_dt.added),
    #                                remove_names=[s for s in transients_dt.removed if s not in self.get_transients_available()]
    #                                )
    #
    #     return DiffTuple(
    #         added=tdt.added,
    #         removed=tdt.removed,
    #     )


    # TODO : "wait_for_it" methods that waits for hte detection of a topic/service on the system
    # TODO : Should return a future so use can decide to wait on it or not
    # TODO : Maybe similar to a async_detect ( hooked up to the detected transient, not the exposed ones )
    # TODO : Exposed interface is for direct control flow => async not really needed
    # TODO : Detect/Update interface is inversed control flow ( from update loop ) => Needed


