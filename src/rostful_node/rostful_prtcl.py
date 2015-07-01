from __future__ import absolute_import

import logging
from collections import namedtuple
"""
Pure python protocol ( only for pipe communication between process )
Designed to have always atomic sensible data on the pipe.
"""

MsgBuild = namedtuple("MsgBuild", "connec_name msg_value")
# rqst : msg_value = None
# resp : msg_value = <msg_instance>

Topic = namedtuple("Topic", "name msg_value")
# Inject :
# rqst : msg_value = <msg_instance>
# resp : msg_value = None
# Extract :
# rqst : msg_value = None
# resp : msg_value = <msg_instance>

Service = namedtuple("Service", "name rqst_value resp_value")
# rqst : rqst_value = <msg_instance_1>, resp_value = None
# resp : rqst_value = <msg_instance_1>, resp_value = <msg_instance>
