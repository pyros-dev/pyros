from __future__ import absolute_import

from . import deffile
from .util import get_json_bool, type_str, load_type

def get_all_msg_types(msg, skip_this=False, type_set=None):
    if type_set is None:
        type_set = set()
    if not skip_this:
        if msg in type_set:
            return type_set
        type_set.add(msg)
    for slot_type in msg._slot_types:
        if '/' not in slot_type:
            continue
        type_set = get_all_msg_types(load_type(slot_type), type_set=type_set)
    return type_set


def get_msg_definitions(msg, skip_this=False):
    type_set = get_all_msg_types(msg, skip_this=skip_this)

    msg_dfns = []
    for msg_type in type_set:
        dfn = deffile.ROSStyleDefinition('msg', type_str(msg_type), ['msg'])
        for field_name, field_type in zip(msg_type.__slots__, msg_type._slot_types):
            dfn.segment(0).append((field_name, field_type))
        msg_dfns.append(dfn)
    return msg_dfns


def get_definitions(services=None, topics=None, actions=None):
    if services is None:
        services = []
    if topics is None:
        topics = []
    if actions is None:
        actions = []
    service_dfns = []
    action_dfns = []
    msg_dfns = []

    type_set = set()
    for service in services:
        dfn = deffile.ROSStyleDefinition('srv', service.get('rostype_name'), ['request', 'response'])
        service_type = load_type(service.get('rostype_name'))
        for field_name, field_type in service.get('srvtype',[None, None])[0].iteritems():
            dfn.segment(0).append((field_name, field_type))
            type_set = get_all_msg_types(service_type._request_class, skip_this=True, type_set=type_set)
        for field_name, field_type in service.get('srvtype',[None, None])[1].iteritems():
            dfn.segment(1).append((field_name, field_type))
            type_set = get_all_msg_types(service_type._response_class, skip_this=True, type_set=type_set)
        service_dfns.append(dfn)

    for action in actions:
        dfn = deffile.ROSStyleDefinition('action', action.rostype_name, ['goal', 'result', 'feedback'])
        for field_name, field_type in zip(action.rostype_goal.__slots__, action.rostype_goal._slot_types):
            dfn.segment(0).append((field_name, field_type))
            type_set = get_all_msg_types(action.rostype_goal, skip_this=True, type_set=type_set)
        for field_name, field_type in zip(action.rostype_result.__slots__, action.rostype_result._slot_types):
            dfn.segment(1).append((field_name, field_type))
            type_set = get_all_msg_types(action.rostype_result, skip_this=True, type_set=type_set)
        for field_name, field_type in zip(action.rostype_feedback.__slots__, action.rostype_feedback._slot_types):
            dfn.segment(2).append((field_name, field_type))
            type_set = get_all_msg_types(action.rostype_feedback, skip_this=True, type_set=type_set)
        action_dfns.append(dfn)

    for topic in topics:
        topic_type = load_type(topic.get('rostype_name'))
        type_set = get_all_msg_types(topic_type, type_set=type_set)

    for msg_type in type_set:
        dfn = deffile.ROSStyleDefinition('msg', type_str(msg_type), ['msg'])
        for field_name, field_type in zip(msg_type.__slots__, msg_type._slot_types):
            dfn.segment(0).append((field_name, field_type))
        msg_dfns.append(dfn)

    return msg_dfns + service_dfns + action_dfns


def manifest(services, topics, actions, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Node'

    if services:
        service_section = deffile.INISection('Services')
        for service_name, service in services.iteritems():
            service_section.fields[service_name] = service.rostype_name
        dfile.add_section(service_section)

    if actions:
        action_section = deffile.INISection('Actions')
        for action_name, action in actions.iteritems():
            action_section.fields[action_name] = action.rostype_name
        dfile.add_section(action_section)

    topics_section = deffile.INISection('Topics')
    publish_section = deffile.INISection('Publishes')
    subscribe_section = deffile.INISection('Subscribes')

    for topic_name, topic in topics.iteritems():
        if topic.allow_sub:
            publish_section.fields[topic_name] = topic.rostype_name
        if topic.allow_pub:
            subscribe_section.fields[topic_name] = topic.rostype_name

    if topics_section.fields:
        dfile.add_section(topics_section)
    if publish_section.fields:
        dfile.add_section(publish_section)
    if subscribe_section.fields:
        dfile.add_section(subscribe_section)

    if full:
        dfns = get_definitions(services=services.itervalues(), topics=topics.itervalues(), actions=actions.itervalues())
        [dfile.add_definition(dfn) for dfn in dfns]

    return dfile


def describe_service(service_name, service, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Service'
    dfile.manifest['Name'] = service_name
    dfile.manifest['Type'] = service.get('rostype_name', 'Unknown')

    if full:
        dfns = get_definitions(services=[service])
        [dfile.add_definition(dfn) for dfn in dfns]

    return dfile


# Interestingly this is used from rostful, to make sense of the data returned by pyros client, not from pyros itself...
# TODO : maybe need to move it ?
# TODO : check, maybe same with some other methods here...
def describe_topic(topic_name, topic, full=False):
    dfile = deffile.DefFile()
    dfile.manifest.def_type = 'Topic'
    dfile.manifest['Name'] = topic_name
    dfile.manifest['Type'] = topic.get('rostype_name', 'Unknown')
    # this is obsolete, now each Topic instance does both...
    #dfile.manifest['Publishes'] = get_json_bool(topic.allow_sub)
    #dfile.manifest['Subscribes'] = get_json_bool(topic.allow_pub)

    if full:
        dfns = get_definitions(topics=[topic])
        [dfile.add_definition(dfn) for dfn in dfns]

    return dfile

