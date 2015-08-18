from __future__ import absolute_import

import logging
from collections import namedtuple
"""
Pure python protocol ( only for pipe communication between process )
Designed to have always atomic sensible data on the pipe.
"""

MsgBuild = namedtuple("MsgBuild", "name msg_content")
# rqst : msg_content = None
# resp : msg_content = <msg_asdict>

Topic = namedtuple("Topic", "name msg_content")
# Inject :
# rqst : msg_content = <msg_asdict>
# resp : msg_content = None
# Extract :
# rqst : msg_content = None
# resp : msg_content = <msg_asdict>

TopicList = namedtuple("TopicList", "name_dict")
TopicInfo = namedtuple('TopicInfo', 'fullname')

Service = namedtuple("Service", "name rqst_content resp_content")
# rqst : rqst_content = <msg_asdict_1>, resp_content = None
# resp : rqst_content = <msg_asdict_1>, resp_content = <msg_asdict>

ServiceList = namedtuple("ServiceList", "name_dict")
ServiceInfo = namedtuple('ServiceInfo', 'fullname')

Namespaces = namedtuple("Namespaces", "namespace_dict")

Interactions = namedtuple("Interactions", "interaction_dict")
