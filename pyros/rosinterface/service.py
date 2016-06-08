from __future__ import absolute_import


import roslib
import rospy

from importlib import import_module
from collections import OrderedDict

from .message_conversion import get_msg, get_msg_dict, populate_instance, extract_values, FieldTypeMismatchException, NonexistentFieldException


# outputs message structure as string (useful ?)
def get_service_srv(service):
    return '\n'.join([
        get_msg(service.rostype_req),
        '---',
        get_msg(service.rostype_resp),
    ])


# outputs message structure as dict
def get_service_srv_dict(service):
    return get_msg_dict(service.rostype_req), get_msg_dict(service.rostype_resp)


class ServiceTuple(object):
    """
    Class representing the service the interface connects to
    """
    def __init__(self, name, type):
        self.name = name
        self.type = type
# Note : for service the connection endpoint is not important
# R1 : service concept is not connection oriented ( client is inexistant until service is used )
# R2 : ROS only keeps the last service provider in master, previous ones are just erased.
# TODO: make that the pickled representation of ServiceBack (check asdict())


class ServiceBack(object):
    """
    ServiceBack is the class handling conversion from Python API to ROS Service
    """
    def __init__(self, service_name, service_type):
        self.name = service_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        service_type_module, service_type_name = tuple(service_type.split('/'))
        roslib.load_manifest(service_type_module)
        srv_module = import_module(service_type_module + '.srv')

        self.rostype_name = service_type
        self.rostype = getattr(srv_module, service_type_name)
        self.rostype_req = getattr(srv_module, service_type_name + 'Request')
        self.rostype_resp = getattr(srv_module, service_type_name + 'Response')

        self.srvtype = get_service_srv_dict(self)
        #rospy.logwarn('srvtype : %r', self.srvtype)

        self.proxy = rospy.ServiceProxy(self.name, self.rostype)

    def cleanup(self):
        pass

    def asdict(self):
        """
        Here we provide a dictionary suitable for a representation of the Topic instance
        the main point here is to make it possible to transfer this to remote processes.
        We are not interested in pickleing the whole class with Subscriber and Publisher
        :return:
        """

        return OrderedDict({
            'name': self.name,
            'fullname': self.fullname,
            'rostype_name': self.rostype_name,
            'srvtype': self.srvtype,
        })

    def call(self, rosreq_content = None):
        try:
            rqst = self.rostype_req()
            populate_instance(rosreq_content, rqst)

            fields = []
            if rosreq_content:
                for slot in rqst.__slots__:
                    fields.append(getattr(rqst, slot))
                fields = tuple(fields)

            resp = self.proxy(*fields)
            resp_content = extract_values(resp)

            return resp_content

        except rospy.ServiceException as e:
            rospy.logerr("[{name}] : service exception {e}".format(name=__name__, e=e))
            raise

        except FieldTypeMismatchException as e:
            rospy.logerr("[{name}] : field type mismatch {e}".format(name=__name__, e=e))
            raise
        except NonexistentFieldException as e:
            rospy.logerr("[{name}] : non existent field {e}".format(name=__name__, e=e))
            raise


