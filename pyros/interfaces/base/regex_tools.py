from __future__ import absolute_import

import logging
import re


# Utility methods to work with regexes
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


def find_first_regex_match(key, regex_candidates):
    """
    find the first regex that match with the key
    :param key: a key to try to match against multiple regex
    :param match_candidates: a list of regexes to check if they match the key
    :return: the first candidate found
    """
    for cand in regex_candidates:
        try:
            pattern = re.compile(cap_match_string(cand))
            if pattern.match(key):
                return cand
        except:
            logging.warn('[ros_interface] Ignoring invalid regex string "{0!s}"!'.format(cand))

    return None


def regex_match_sublist(regex, match_candidates):
    """
    Filter the match_candidates list to return only the candidate that match the regex
    :param regex: a regex used to filter the list of candidates
    :param match_candidates: the list of candidates
    :return: the filtered list of only the candidates that match the regex
    """
    matches = []
    try:
        pattern = re.compile(cap_match_string(regex))
        matches = [cand for cand in match_candidates if pattern.match(cand)]
    except:
        logging.warn('[ros_interface] Ignoring invalid regex string "{0!s}"!'.format(regex))
    return matches


def regexes_match_sublist(regexes, match_candidates):
    """
    Filter the match_candidates list to return only the candidate that match the regex
    :param regexes: a list of regex used to filter the list of candidates
    :param match_candidates: the list of candidates
    :return: the filtered list of only the candidates that match the regex
    """

    return [match for sublist in [regex_match_sublist(rgx, match_candidates) for rgx in regexes] for match in sublist]