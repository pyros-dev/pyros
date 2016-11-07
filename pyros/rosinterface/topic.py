# from __future__ import absolute_import
#
# import time
#
# import roslib
# import rospy
#
# from importlib import import_module
# from collections import deque, OrderedDict
#
# from ..baseinterface import TransientIf
# from .message_conversion import get_msg, get_msg_dict, populate_instance, extract_values, FieldTypeMismatchException
#
# from .publisher import PublisherBack
# from .subscriber import SubscriberBack
#
#
# class TopicBackTimeout(Exception):
#     pass
#
#
# # Maybe have a global method that generate a context manager to interface with it...
# class TopicTuple(object):
#     def __init__(self, name, type, endpoints):
#         self.name = name
#         self.type = type
#         self.endpoints = endpoints
# # Note : for topic the connection endpoint is important.
# # We can have multiple subscribers and publishers, from different node.
# # We need to know if we can drop our interface when we receive a difference ( only some pub|sub lost )
# # => we need to track endpoints
# # TODO: make that the pickled representation of TopicBack (check asdict())
#
#
# class TopicBack(TransientIf):
#     """
#     TopicBack is the class handling conversion from Python to ROS Topic
#     This is a basic temporary implementation to provide same API as previous 0.2 versions
#     Goal is to provide different configuration for publishers and subscribers
#     => this should disappear with pyros 0.3
#     """
#
#     # We need some kind of instance count here since system state returns only one node instance
#     # as publisher for multiple publisher in this process.
#     #
#     # This is used in ros_interface update for added puba/subs to determine
#     # if we previously started publishing / subscribing to this topic
#     # usually the count will be just 1, but more is possible during tests
#     #
#     # This is also used in ros_interface update for removed pubs/subs to determine
#     # if we previously stopped publishing / subscribing to this topic
#     # usually the count will be just 1, but more is possible during tests
#
#
#     # TODO: maybe contextmanager to make sure unregister always happens if we create it ?
#     # Should be fine since hte interface is in loop...
#     # this might help keep track of the interface created pub/sub, and help keep things tidy...
#
#     @staticmethod
#     def _create_pub(name, rostype, *args, **kwargs):
#         """
#         Creating a publisher and adding it to the pub instance count.
#         Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
#         :return: the ros publisher
#         """
#         return PublisherBack._create(name, rostype, *args, **kwargs)
#
#
#     @staticmethod
#     def _remove_pub(pub):
#         """
#         Removing a publisher and substracting it from the pub instance count.
#         :return: None
#         """
#         return PublisherBack._remove(pub)
#
#     @staticmethod
#     def _create_sub(name, rostype, topic_callback, *args, **kwargs):
#         """
#         Creating a subscriber and adding it to the pub instance count.
#         Static and Functional style so we can call it from anywhere (tests).
#         Useful in case we need multiple similar publisher in one process ( tests, maybe future cases )
#         :return: the subscriber
#         """
#         return SubscriberBack._create(name, rostype, topic_callback, *args, **kwargs)
#
#     @staticmethod
#     def _remove_sub(sub):
#         """
#         Removing a subscriber and substracting it from the sub instance count.
#         :return: None
#         """
#         return SubscriberBack._remove(sub)
#
#
#     # We need this because we cannot really trust get_num_connections() (updated only after message is published)
#     # USED !
#     @staticmethod
#     def get_pub_impl_ref_count(name):
#         """
#         Return the number of internal references to that publisher implementation
#         Useful to keep track of multiple instance of publisher in one process(for tests for examples)
#         In all cases we have only one implementation (rospy), one interface (pyros), but external code in same process
#         can add references, that we want to be counted as part of system, and not part of interface...
#         TODO : test nodelets...
#         :param name: name of the topic
#         :return: impl.ref_count() for that topic
#         """
#
#         return PublisherBack.pool.get_impl_ref_count(name)
#
#     # USED !
#     @staticmethod
#     def get_sub_impl_ref_count(name):
#         return SubscriberBack.pool.get_impl_ref_count(name)
#
#     # We still need the list of connections to find out if this topic is currently in system or not...
#     # USED !
#     @staticmethod
#     def get_pub_impl_connections(name):
#         """
#         Return the number of internal conection to that publisher (from the subscriber point of view)
#         Useful to keep track of publisher in multiple processes
#         :param name: name of the topic
#         :return: impl.get_stats_info() for that topic SUBSCRIBER
#         """
#         # TODO : separate pub and sub
#         return PublisherBack.pool.get_impl_connections(name)
#
#     # USED !
#     @staticmethod
#     def get_sub_impl_connections(name):
#         return SubscriberBack.pool.get_impl_connections(name)
#
#     def __init__(self, topic_name, topic_type, sub_queue_size=1, pub_start_timeout=2):
#
#         super(TopicBack, self).__init__(topic_name, topic_type)
#
#         # We need to create sub first
#         self.if_sub = SubscriberBack(topic_name, topic_type, sub_queue_size)
#         print("REF DEBUG : {name} ref_count : {ref_count}".format(name=topic_name, ref_count=self.if_sub.topic.impl.ref_count))
#
#         self.if_pub = PublisherBack(topic_name, topic_type)
#         print("REF DEBUG : {name} ref_count : {ref_count}".format(name=topic_name, ref_count=self.if_pub.topic.impl.ref_count))
#
#         # Here making sure the publisher is actually connected
#         # before returning to ensure RAII
#         # Note : The sub should be immediately usable.
#         start = time.time()
#         timeout = pub_start_timeout
#         # REMEMBER sub connections is a list of pubs and pub connections is the list of subs...
#         while time.time() - start < timeout and (
#             self.if_pub.topic.get_num_connections() < 1  # at least our own subscriber should connect here
#         ):
#             rospy.rostime.wallsleep(0.1)
#         if not time.time() - start < timeout:
#             raise TopicBackTimeout()
#         # Note : we cannot do that in PublisherBack, because we do not know if a subscriber even exist in the system...
#
#     def cleanup(self):
#         """
#         Launched when we want to whithhold this interface instance
#         :return:
#         """
#         # cleanup pub and sub, so we can go through another create / remove cycle properly
#         # Sub needs to be removed first if we want the pub removal to not throw errors
#         self.if_sub.cleanup()
#         # The pub is not going to detect the sub is gone, unless a message is send
#         # which will refresh the connections list.
#         # => we need to remove it without waiting
#         self.if_pub.cleanup()
#
#         super(TopicBack, self).cleanup()
#
#     def asdict(self):
#         """
#         Here we provide a dictionary suitable for a representation of the Topic instance
#         the main point here is to make it possible to transfer this to remote processes.
#         We are not interested in pickleing the whole class with Subscriber and Publisher
#         :return:
#         """
#         # simple merge for now...
#         d = self.if_pub.asdict()
#         d.update(self.if_sub.asdict())
#         return d
#
#     # TMP...
#     @staticmethod
#     def get_all_pub_interfaces():
#         return PublisherBack.pool.get_all_interfaces()
#
#     @staticmethod
#     def get_all_sub_interfaces():
#         return SubscriberBack.pool.get_all_interfaces()
#
#
#     # TMP... IMPL1
#     @staticmethod
#     def get_interface_only_topics(): #_by_implconn():
#         if_topics = {}
#         if_pubs = TopicBack.get_interface_only_publishers_by_implconn()
#         if_subs = TopicBack.get_interface_only_subscribers_by_implconn()
#         for node in if_pubs:
#             if_topics[node] = if_pubs.get('publishers',{})
#         for node in if_subs:
#             for t, if_only in if_subs[node].get('subscribers', {}).iteritems():
#                 if t in if_topics[node]:
#                     if_topics[node][t] = if_only and if_topics[node][t]  # CAREFUL with lazy evaluation
#                 else:
#                     if_topics[node][t] = if_only
#
#         #print("IF TOPICS : {0}".format(if_topics))
#
#         # inversing mapping, filtering only active interface, and checking node unicity (to match other APIs)
#         inv_if_topics = {}
#         for node, topicdict in if_topics.iteritems():
#             for t in topicdict:
#                 if topicdict[t]:
#                     keys = inv_if_topics.setdefault(t, set())
#                     keys.add(node)
#
#         #print("REMAPPED IF TOPICS : {0}".format(inv_if_topics))
#
#         return inv_if_topics
#
#
#     # # TMP... IMPL2
#     # @staticmethod
#     # def get_interface_only_topics_by_ifparam():
#     #     return TopicBack.get_interface_only_publishers_by_ifparam() + TopicBack.get_interface_only_subscribers()
#
#     # TMP... IMPL 1
#     @staticmethod
#     def get_interface_only_publishers_by_implconn():
#
#         # First we get all publisher interfaces
#         pubs_if = TopicBack.get_all_pub_interfaces()
#         #print("\nget_interface_only_publishers_by_implconn IN: {0}".format(pubs_if))
#
#         # Second we "turn off" the ones that have non-pyros-interface connections to mark them as needed
#         for node_name, node_data in pubs_if.iteritems():
#             node_pubs = node_data.get('publishers',{})
#             for pub_name in node_pubs:
#                 if node_pubs[pub_name]:
#                     # REMINDER : this list the connections ie, the list of subscribers
#                     # CAREFUL : this is not really reliable (publisher doesnt update until it publish a message...)
#                     pub_conns = TopicBack.get_pub_impl_connections(pub_name)
#                     # Then we need to turn off the ones not from this process interface
#                     nonif_pub_conns_node = [c[1] for c in pub_conns if c[1] not in pubs_if] # we need to check for ALL node names
#                     if nonif_pub_conns_node:  # if the list is not empty
#                         # it means we have connections -> we need to set it to False here (not interfaced)
#                         node_pubs[pub_name] = False
#
#         # Third we "turn off" the ones that have more than one ref in this process
#         mypubs = pubs_if.get(rospy.get_name(), {}).get('publishers', {})
#         for pub_name in mypubs:
#             if mypubs[pub_name] and TopicBack.get_pub_impl_ref_count(pub_name) > 1:
#                 mypubs[pub_name] = False
#
#         #print("get_interface_only_publishers_by_implconn OUT: {0}".format(pubs_if))
#         return pubs_if
#
#         # TMP... IMPL 1
#     @staticmethod
#     def get_interface_only_subscribers_by_implconn():
#
#         # First we get all publisher interfaces
#         subs_if = TopicBack.get_all_sub_interfaces()
#         #print("\nget_interface_only_subscribers_by_implconn IN: {0}".format(subs_if))
#
#         # Second we "turn off" the ones that have non-pyros-interface connections to mark them as needed
#         for node_name, node_data in subs_if.iteritems():
#             node_subs = node_data.get('subscribers', {})
#             for sub_name in node_subs:
#                 if node_subs[sub_name]:
#                     # REMINDER : this list the connections ie, the list of publishers
#                     sub_conns = TopicBack.get_sub_impl_connections(sub_name)
#                     # Then we need to turn off the ones not from this process interface
#                     subs_if_uri = [subs_if[n].get('uri', "") for n in subs_if]  # we need to check for ALL node uris
#                     nonif_sub_conns_node = [c[1] for c in sub_conns if c[1] not in subs_if_uri]
#                     if nonif_sub_conns_node:  # if the list is not empty
#                         # it means we have connections -> we need to set it to False here (not interfaced)
#                         node_subs[sub_name] = False
#
#         # Third we "turn off" the ones that have more than one ref in this process
#         mysubs = subs_if.get(rospy.get_name(), {}).get('subscribers', {})
#         for sub_name in mysubs:
#             if mysubs[sub_name] and TopicBack.get_sub_impl_ref_count(sub_name) > 1:
#                 mysubs[sub_name] = False
#
#         #print("get_interface_only_subscribers_by_implconn OUT: {0}".format(subs_if))
#         return subs_if
#
#
#     # # TMP... IMPL 2
#     # @staticmethod
#     # def get_interface_only_publishers_by_ifparam():
#     #
#     #     # First we get all publisher interfaces
#     #     pubs_if = TopicBack.get_all_pub_interfaces()
#     #     print("get_interface_only_publishers_by_ifparam IN : {0}".format(pubs_if))
#     #
#     #     # Second we verify which ones need to be kept, based on this process alone.
#     #     if_only_list = {}
#     #     for node, tifs in pubs_if.iteritems():
#     #         if node == rospy.get_name():
#     #             # For our interface, only keep if the ref_count is <= 1
#     #             # More means we have another pub|sub instance somewhere, and we should reflect it as part of the system.
#     #             # This is used by tests for example, to simulate a pub|sub without having to spawn a different process.
#     #             # But IT IS NOT A NORMAL USECASE. For pyros to work multiprocess it needs to stick to ONE interface instance per process.
#     #             for tifname, tifon in tifs.get('interfaces', {}).iteritems():
#     #                 # In theory : tifon is False <=> ref_count == 0
#     #                 # but if not tifon, we probably want to keep the topic in the list...
#     #                 if tifon and TopicBack.get_pub_impl_ref_count(tifname) <= 1 and TopicBack.get_sub_impl_ref_count(tifname) <= 1:
#     #                     # Note we could also : self.topics_pool.available[tname].get_impl_ref_count()
#     #
#     #                     # now checking non-pyros connections :
#     #                     pub_conns = TopicBack.get_pub_impl_connections(tifname)
#     #                     if node not in [c[1] for c in pub_conns]]
#     #                     print("RESULT : {0}".format(non_pubif_nodes))
#     #
#     #                     sub_conns = TopicBack.get_sub_impl_connections(tifname)
#     #
#     #
#     #
#     #
#     #                     # We can drop this node from topics list
#     #                     if_only_list[tifname] = if_only_list.get(tifname, []) + [node]
#     #                     # elif not tifon:
#     #                     #     # useful to drop lingering publishers
#     #                     #     drop_list[tifname] += [node]
#     #                     print("{tifname} is interface only because impl_ref_count is pub: {pub_ref_count}, sub: {sub_ref_count} ".format(tifname = tifname, pub_ref_count= TopicBack.get_pub_impl_ref_count(tifname), sub_ref_count= TopicBack.get_sub_impl_ref_count(tifname)))
#     #         else:
#     #             # For other pyros interface we can only assume they intend to drop the interface as soon as the system lose that topic
#     #             # And we are not interested in communication between pyros instances.
#     #             # We can drop this node from topics list, for all topics
#     #             for tifname, tifon in tifs.get('interfaces', {}).iteritems():
#     #                 if tifon:
#     #                     if_only_list[tifname] = if_only_list.get(tifname, []) + [node]
#     #                     # elif not tifon: # if not tifon, the presence of the node does NOT reflect an interface...
#     #                     #     # but maybe useful to drop lingering publishers ?
#     #                     #     drop_list[tifname] += [node]
#     #                     print("{tifname} is interface only because node is {node} != {mynode}".format(tifname=tifname, node=node, mynode = rospy.get_name() ))
#     #
#     #     # Second we check existing connections to these interfaces, and if there is something else connected, we cannot drop it
#     #     # Extracting the dict of topics that are only interface, in this process, and not from the system.
#     #     drop_list = {}
#     #     for name, nodes in if_only_list.iteritems():
#     #         pub_conns = TopicBack.get_pub_impl_connections(name)
#     #
#     #         print("FROM: {0}".format(nodes))
#     #         print("FILTERING OUT PUB CONNS NODES: {0}".format([c[1] for c in pub_conns]))
#     #         non_pubif_nodes = [n for n in nodes if n not in [c[1] for c in pub_conns]]
#     #         print("RESULT : {0}".format(non_pubif_nodes))
#     #
#     #         sub_conns = TopicBack.get_sub_impl_connections(name)
#     #
#     #         print("FROM: {0}".format([topics_if[n].get('uri', "") for n in nodes]))
#     #         print("FILTERING OUT SUB CONNS URI : {0}".format([c[1] for c in sub_conns]))
#     #         non_subif_nodes = [n for n in nodes if topics_if[n].get('uri', "") not in [c[1] for c in sub_conns]]
#     #         print("RESULT : {0}".format(non_subif_nodes))
#     #
#     #         if non_pubif_nodes or non_subif_nodes:
#     #             drop_list[name] = non_subif_nodes + non_subif_nodes
#     #
#     #     print("TOPIC EARLY DROPLIST : {0}".format(drop_list))
#     #     return drop_list
#
#
#     def publish(self, msg_content):
#         """
#         Publishes a message to the topic
#         :return the actual message sent if one was sent, None if message couldn't be sent.
#         """
#         return self.if_pub.publish(msg_content=msg_content)
#
#     def get(self, num=0, consume=False):
#         return self.if_sub.get(num=num, consume=consume)
#
#     #returns the number of unread message
#     def unread(self):
#         return self.if_sub.unread()
#
#     def set_empty_callback(self, cb):
#         return self.if_sub.unread()
#
#     def topic_callback(self, msg):
#         return self.if_sub.topic_callback(msg=msg)
#
