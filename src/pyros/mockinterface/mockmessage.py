import __builtin__
import cPickle
from collections import namedtuple

# TODO : use basemsg and develop a generic way of converting python type to custom type

# type map specifying conversion
# basic types should be python builtin types
# if not in there : assumed to be a custom type, convertible to a dict.
type_map = {
   "bool":    ["bool"],
   "int":     ["int", "float"],  # can convert int to float
   "float":   ["float"],
   "str":     ["str", "unicode"],  # convert str to unicode to follow python 3 default behavior
   "unicode": ["unicode"],
   "list":    ["list"],
   "tuple":   ["tuple", "list"],  # can convert tuple to list
}

# Starting with whatever ROS needs, but we could extend this
primitive_types = [bool, int, long, float, str, unicode]
composed_types = [list, tuple]

# defining Mock message types using namedtuple to keep things small
StatusMsg = namedtuple("StatusMsg", "error code message")

# defining Exceptions
class NonexistentFieldException(Exception):
    def __init__(self, oridict, basetype, message):
        Exception.__init__(self, "Trying to convert {0!s} to Message type {1!s} triggered {2!s}".format(oridict, basetype, message))


class FieldTypeMismatchException(Exception):
    def __init__(self, roottype, fields, expected_type, found_type):
        if roottype == expected_type:
            Exception.__init__(self, "Expected a Python object for type {0!s} but received a {1!s}".format(roottype, found_type))
        else:
            Exception.__init__(self, "{0!s} message requires a {1!s} for field {2!s}, but got a {3!s}".format(roottype, expected_type, '.'.join(fields), found_type))


def extract_values(inst):
    """
    :param inst: the instance
    :return: python values extracted from the instance
    """
    # inst should already be python
    return inst


def populate_instance(msg, inst):
    """
    :param msg: contains the values to use to populate inst.
    :param inst: message class instance to populate.
    :return: an instance of the provided message class, with its fields populated according to the values in msg
    """
    return _to_inst(msg, type(inst).__name__, type(inst).__name__, inst)


def _to_inst(msg, ptype, roottype, inst=None, stack=None):

    # Check to see whether this is a primitive type
    if stack is None:
        stack = []
    if ptype in __builtin__.__dict__ and (
        __builtin__.__dict__[ptype] in primitive_types or
        __builtin__.__dict__[ptype] in composed_types
    ):
        # Typecheck the msg
        msgtype = type(msg)

        if msgtype in primitive_types and ptype in type_map[msgtype.__name__]:
            return __builtin__.__dict__[ptype](msg)
        elif msgtype in composed_types and ptype in type_map[msgtype.__name__]:
            # Call to _to_inst for every element of the list/tuple
            def recurse_iter(msg):
                for e in msg:  # we do this with yield to get an iteratable and build the tuple/list at once
                    yield _to_inst(e, type(e).__name__, roottype, None, stack)

            return __builtin__.__dict__[ptype](recurse_iter(msg))

        raise FieldTypeMismatchException(roottype, stack, ptype, msgtype)

    # Otherwise, the type has to be a custom type, so msg must be a dict
    if type(msg) is not dict:
        raise FieldTypeMismatchException(roottype, stack, ptype, type(msg))

    # and ptype should be able to build with same fields.

    # modifying dict with dict comprehension
    instmsg = dict((k, _to_inst(v, type(v).__name__, roottype, None, stack)) for k, v in msg.items())

    # using ** to get dict content as named args for the namedtuple
    try:
        inst = globals()[ptype](**instmsg)
    except TypeError, e:
        raise NonexistentFieldException(msg, ptype, e.message)
    return inst
