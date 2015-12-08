from __future__ import absolute_import

import logging
import sys
import collections
import re
import abc
from functools import partial


class BaseInterface(object):
    """
    BaseInterface.
    Assumption : we only deal with absolute names here. The users should resolve them
    """
    __metaclass__ = abc.ABCMeta
    DiffTuple = collections.namedtuple("DiffTuple", " added removed ")

    @staticmethod
    def cap_match_string(match):
        """
        Attach beginning of line and end of line characters to the given string.
        Ensures that raw topics like /test do not match other topics containing
        the same string (e.g. /items/test would result in a regex match at char 7)
        regex goes through each position in the string to find matches - this
        forces it to consider only the first position
        :param match: a regex
        :return:
        """
        return '^' + match + '$'
        
    @staticmethod
    def find_first_regex_match(key, regex_candidates):
        """
        find the first regex that match with the key
        :param key: a key to try to match against multiple regex
        :param match_candidates: a list of regexes to check if they match the key
        :return: the first candidate found
        """
        for cand in regex_candidates:
            try:
                pattern = re.compile(BaseInterface.cap_match_string(cand))
                if pattern.match(key):
                    return cand
            except:
                logging.warn('[ros_interface] Ignoring invalid regex string "%s"!' % cand)

        return None

    @staticmethod
    def regex_match_sublist(regex, match_candidates):
        """
        Filter the match_candidates list to return only the candidate that match the regex
        :param regex: a regex used to filter the list of candidates
        :param match_candidates: the list of candidates
        :return: the filtered list of only the candidates that match the regex
        """
        matches = []
        try:
            pattern = re.compile(BaseInterface.cap_match_string(regex))
            matches = [cand for cand in match_candidates if pattern.match(cand)]
        except:
            logging.warn('[ros_interface] Ignoring invalid regex string "%s"!' % regex)
        return matches

    @staticmethod
    def _update_transients(add_names, remove_names, transient_desc, resolved_dict, type_resolve_func, class_build_func, *class_build_args, **class_build_kwargs):
        """
        Adds transients (service, topic, etc) named in add_names and removes transients named in remove_names  exposed transients resolved_dict
        :param transient_desc: a string describing the transient type ( service, topic, etc. )
        :param add_names: the names of the transients to add
        :param remove_names: the names of the transients to remove
        :param resolved_dict: the list to add the resolved transient(s) to
        :param get_list_func: the function to get the list of all transients currently available to be exposed
        :param type_resolve_func: the function to retrieve the message type used by the transient
        :param class_build_func: the class constructor function to be able to create a resolved transient
        :param class_build_args: the args to pass to the resolved transient constructor
        :return: the list of transients exposed
        """
        # Important : no effect if names is empty list. only return empty. functional style.
        added = []
        removed = []
        for tst_name in [tst for tst in add_names if tst not in resolved_dict.keys()]:
            try:
                ttype = type_resolve_func(tst_name)  # should return None if error - TODO : handle (and reraise) exception !!
                if ttype is not None:  # transient can be resolved
                    resolved_dict[tst_name] = class_build_func(tst_name, ttype, *class_build_args, **class_build_kwargs)
                    added += [tst_name]
                    logging.info("[{name}] Interfacing with {desc} {transient}".format(name=__name__, desc=transient_desc, transient=tst_name))
            except Exception, e:
                logging.warn("[{name}] Cannot interface with {desc} {transient} : {exc}".format(name=__name__, desc=transient_desc, transient=tst_name, exc=e))
                exc_info = sys.exc_info()
                raise exc_info[0], exc_info[1], exc_info[2]

        for tst_name in [tst for tst in remove_names if tst in resolved_dict.keys()]:
            logging.info("[{name}] Removing {desc} {transient}".format(name=__name__, desc=transient_desc, transient=tst_name))
            resolved_dict.pop(tst_name, None)
            removed += [tst_name]

        return BaseInterface.DiffTuple(added, removed)

    @staticmethod
    def _expose_transients_regex(regexes, transient_desc, regex_set, resolved_dict,
                            get_list_func, type_resolve_func,
                            class_build_func, *class_build_args, **class_build_kwargs):
        """
        Exposes a list of transients regexes. resolved transients not matching the regexes will be removed.
        :param transient_desc: a string describing the transient type ( service, topic, etc. )
        :param regexes: the list of regex to filter the transient to add
        :param resolved_dict: the list to add the resolved transient(s) to
        :param get_list_func: the function to get the list of all transients currently available to be exposed
        :param add_func: the function used to add the transient to the list of exposed ones
        :param rem_func: the function used to remove the transient form the list of exposed ones
        :return: the list of transients exposed
        """
        # Important : no effect if names is empty list. only return empty diff. functional style.

        add_names = []
        rem_names = []
        # look through the new service names received by reconfigure, and add
        # those services which are not in the existing service args
        for tst_regex in [r for r in regexes if not r in regex_set]:
            regex_set.add(tst_regex)
            logging.info('[{name}] Exposing {desc} regex : {regex}'.format(name=__name__, desc=transient_desc, regex=tst_regex))
            add_names = BaseInterface.regex_match_sublist(tst_regex, get_list_func())

        # look through the current service args and delete those values which
        # will not be valid when the args are replaced with the new ones. run on
        # a copy so that we will remove from the original without crashing
        for tst_regex in [r for r in regex_set if not r in regexes]:
            rem_names = [n for n in BaseInterface.regex_match_sublist(tst_regex, resolved_dict.keys()) if BaseInterface.find_first_regex_match(n, regexes) is None]
            logging.info('[{name}] Withholding {desc} regex : {regex}'.format(name=__name__, desc=transient_desc, regex=tst_regex))
            regex_set.remove(tst_regex)

        return BaseInterface._update_transients(
                                add_names,
                                rem_names,
                                transient_desc,
                                resolved_dict,
                                type_resolve_func,
                                class_build_func,
                                *class_build_args,
                                **class_build_kwargs
                             )


    @staticmethod
    def _transient_change_detect(regex_set, transient_desc, resolved_dict, last_got_set, get_list_func, type_resolve_func, class_build_func, *class_build_args, **class_build_kwargs):
        """
        This should be called when we want to detect a change in the status of the system regarding the transient list
        """

        transient_detected = set(get_list_func())
        new_tsts = transient_detected - last_got_set
        lost_tsts = last_got_set - transient_detected

        if not new_tsts and not lost_tsts:
            return BaseInterface.DiffTuple([], [])

        added_tst_lst = [t for t in list(set(new_tsts)) if BaseInterface.find_first_regex_match(t, regex_set) is not None]
        removed_tst_lst = [t for t in list(set(lost_tsts)) if BaseInterface.find_first_regex_match(t, regex_set) is not None]

        dt = BaseInterface._update_transients(
                    added_tst_lst,
                    removed_tst_lst,
                    transient_desc,
                    resolved_dict,
                    type_resolve_func,
                    class_build_func,
                    *class_build_args,
                    **class_build_kwargs
                 )

        if lost_tsts:
            last_got_set.difference_update(lost_tsts)
        if new_tsts:
            last_got_set.update(new_tsts)

        return dt

    # Abstract methods to override for services ("RPC / request / call type" communication channel)
    @abc.abstractmethod
    def get_svc_list(self):  # function returning all services available on the system
        return

    @abc.abstractmethod
    def service_type_resolver(self, service_name):  # function resolving the type of a service
        """
        :param service_name: the name of the service
        :return: returns None if the type cannot be found. Properly except in all other unexpected events.
        """
        return

    @abc.abstractmethod
    def ServiceMaker(self, service_name, service_type):  # the service class implementation
        return

    # Abstract methods to override for topics ("pub/sub type" communication channel)
    @abc.abstractmethod
    def get_topic_list(self):  # function returning all topics available on the system
        return

    @abc.abstractmethod
    def topic_type_resolver(self, topic_name):  # function resolving the type of a topic
        """
        :param topic_name: the name of the topic
        :return: returns None if the topic cannot be found. Properly except in all other unexpected events.
        """
        return

    @abc.abstractmethod
    def TopicMaker(self, topic_name, topic_type, *args, **kwargs):  # the topic class implementation
        return

    # Abstract methods to override for params ( "global" get/set objects )
    @abc.abstractmethod
    def get_param_list(self):  # function returning all topics available on the system
        return

    @abc.abstractmethod
    def param_type_resolver(self, param_name):  # function resolving the type of a topic
        return

    @abc.abstractmethod
    def ParamMaker(self, param_name, param_type):  # the topic class implementation
        return

    def __init__(self, services, topics, params):
        # Current services topics and actions exposed, i.e. those which are
        # active in the system.
        self.services = {}
        self.topics = {}
        self.params = {}
        self.actions = {}
        # Currently we ensure that all
        # the sets contain strings which do not have a leading slash.

        # Last requested services topics and actions to be exposed, received
        # from a reconfigure request. Topics which match topics containing
        # wildcards go in here after they are added, but when a new reconfigure
        # request is received, they disappear. The value of the topic and action
        # dicts is the number of instances that that that item has, i.e. how
        # many times the add function has been called for the given key.
        self.services_args = set()
        self.params_args = set()
        self.topics_args = set()

        # to allow comparison with previously detected services / topics/ params
        self.last_services_detected = set()
        self.last_topics_detected = set()
        self.last_params_detected = set()

        # Building an interface dynamically based on the generic functional implementation
        # use is mostly internal ( and child classes )
        self.update_services = partial(BaseInterface._update_transients,
                                       transient_desc="service",
                                       resolved_dict=self.services,
                                       type_resolve_func=self.service_type_resolver,
                                       class_build_func=self.ServiceMaker,
                                       )
        self.update_services.__name__ = "update_services"
        self.update_services.__doc__ = """
        Adds / Removes services from the list of Services exposed
        """

        self.expose_services = partial(BaseInterface._expose_transients_regex,
                                       transient_desc="service",
                                       regex_set=self.services_args,
                                       resolved_dict=self.services,
                                       get_list_func=self.get_svc_list,
                                       type_resolve_func=self.service_type_resolver,
                                       class_build_func=self.ServiceMaker,
                                       )
        self.expose_services.__doc__ = """
        """

        self.services_change_detect = partial(BaseInterface._transient_change_detect,
                                              transient_desc="service",
                                              regex_set=self.services_args,
                                              resolved_dict=self.services,
                                              last_got_set=self.last_services_detected,
                                              get_list_func=self.get_svc_list,
                                              type_resolve_func=self.service_type_resolver,
                                              class_build_func=self.ServiceMaker,
                                              )
        self.services_change_detect.__doc__ = """
        """

        self.update_topics = partial(BaseInterface._update_transients,
                                     transient_desc="topic",
                                     resolved_dict=self.topics,
                                     type_resolve_func=self.topic_type_resolver,
                                     class_build_func=self.TopicMaker,
                                     )
        self.update_topics.__name__ = "update_topics"
        self.update_topics.__doc__ = """
        Adds / Removes topics from the list of Topics exposed
        """

        self.expose_topics = partial(BaseInterface._expose_transients_regex,
                                     transient_desc="topic",
                                     regex_set=self.topics_args,
                                     resolved_dict=self.topics,
                                     get_list_func=self.get_topic_list,
                                     type_resolve_func=self.topic_type_resolver,
                                     class_build_func=self.TopicMaker,
                                     )

        self.topics_change_detect = partial(BaseInterface._transient_change_detect,
                                            transient_desc="topic",
                                            regex_set=self.topics_args,
                                            resolved_dict=self.topics,
                                            last_got_set=self.last_topics_detected,
                                            get_list_func=self.get_topic_list,
                                            type_resolve_func=self.topic_type_resolver,
                                            class_build_func=self.TopicMaker,
                                            )

        self.update_params = partial(BaseInterface._update_transients,
                                     transient_desc="param",
                                     resolved_dict=self.params,
                                     type_resolve_func=self.param_type_resolver,
                                     class_build_func=self.ParamMaker,
                                     )
        self.update_params.__name__ = "update_params"
        self.update_params.__doc__ = """
        Adds / Removes topics from the list of Topics exposed
        """

        self.expose_params = partial(BaseInterface._expose_transients_regex,
                                     transient_desc="param",
                                     regex_set=self.params_args,
                                     resolved_dict=self.params,
                                     get_list_func=self.get_param_list,
                                     type_resolve_func=self.param_type_resolver,
                                     class_build_func=self.ParamMaker,
                                     )

        self.params_change_detect = partial(BaseInterface._transient_change_detect,
                                            transient_desc="topic",
                                            regex_set=self.params_args,
                                            resolved_dict=self.params,
                                            last_got_set=self.last_params_detected,
                                            get_list_func=self.get_param_list,
                                            type_resolve_func=self.param_type_resolver,
                                            class_build_func=self.ParamMaker,
                                            )

        self.reinit(services, topics, params)

        # First update() will re-detect previously found names, but when trying to update, no duplicates should happen.
        # This should be taken care of in the abstract transient functional interface

    def reinit(self, services=None, topics=None, params=None):
        """
        :param services: list of services to expose. None means no change, keep current config.
        :param topics: list of topics to expose. None means no change, keep current config.
        :param params: list of params to expose. None means no change, keep current config.
        :return: the difference between the transient previously exposed and recently exposed
        """
        if services is not None:
            sdt = self.expose_services(services)
        else:
            sdt = BaseInterface.DiffTuple([], [])  # no change in exposed services

        if topics is not None:
            tdt = self.expose_topics(topics)
        else:
            tdt = BaseInterface.DiffTuple([], [])  # no change in exposed services

        if params is not None:
            pdt = self.expose_params(params)
        else:
            pdt = BaseInterface.DiffTuple([], [])  # no change in exposed services

        return BaseInterface.DiffTuple(
            added=sdt.added+tdt.added+pdt.added,
            removed=sdt.removed+tdt.removed+pdt.removed
        )

    def update(self):
        """
        :return: the difference between the transients recently added/removed
        """
        sdt = self.services_change_detect()
        tdt = self.topics_change_detect()
        pdt = self.params_change_detect()

        return BaseInterface.DiffTuple(
            added=sdt.added+tdt.added+pdt.added,
            removed=sdt.removed+tdt.removed+pdt.removed
        )

    # TODO : "wait_for_it" methods that waits for hte detection of a topic/service on the system
    # TODO : Should return a future so use can decide to wait on it or not
    # TODO : Maybe similar to a async_detect ( hooked up to the detected transient, not the exposed ones )
    # TODO : Exposed interface is for direct control flow => async not really needed
    # TODO : Detect/Update interface is inversed control flow ( from update loop ) => Needed
