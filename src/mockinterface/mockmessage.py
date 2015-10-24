import __builtin__
import cPickle

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


class NonexistentFieldException(Exception):
    def __init__(self, basetype, fields):
        Exception.__init__(self, "Message type %s does not have a field %s" % (basetype, '.'.join(fields)))


class FieldTypeMismatchException(Exception):
    def __init__(self, roottype, fields, expected_type, found_type):
        if roottype == expected_type:
            Exception.__init__(self, "Expected a Python object for type %s but received a %s" % (roottype, found_type))
        else:
            Exception.__init__(self, "%s message requires a %s for field %s, but got a %s" % (roottype, expected_type, '.'.join(fields), found_type))


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


def _to_inst(msg, ptype, roottype, inst=None, stack=[]):

    # Check to see whether this is a primitive type
    if __builtin__.__dict__[ptype] in primitive_types or __builtin__.__dict__[ptype] in composed_types:
        # Typecheck the msg
        msgtype = type(msg)

        if msgtype in primitive_types and ptype in type_map[msgtype.__name__]:
            inst = __builtin__.__dict__[ptype](msg)
            return inst
        elif msgtype in composed_types and ptype in type_map[msgtype.__name__]:
            # Call to _to_inst for every element of the list/tuple
            inst = __builtin__.__dict__[ptype](len(msg))
            for i, e in enumerate(msg):
                inst[i] = _to_inst(e, type(e).__name__, roottype, None, stack)
            return inst

        raise FieldTypeMismatchException(roottype, stack, ptype, msgtype)

    # Otherwise, the type has to be a custom type, so msg must be a dict
    if inst is None:
        inst = __builtin__.__dict__[ptype]

    return _to_object_inst(msg, ptype, roottype, inst, stack)

def _to_object_inst(msg, ptype, roottype, inst, stack):
    # Typecheck the msg
    if type(msg) is not dict:
        raise FieldTypeMismatchException(roottype, stack, ptype, type(msg))

    inst_fields = dict((name, getattr(inst, name)) for name in dir(inst) if not name.startswith('__'))
    # Be careful about inheritance though, not sure if it works yet...
    # TODO : test it

    for field_name in msg:
        # Add this field to the field stack
        field_stack = stack + [field_name]

        # Raise an exception if the msg contains a bad field
        if not field_name in inst_fields:
            raise NonexistentFieldException(roottype, field_stack)

        field_ptype = inst_fields[field_name]
        field_inst = getattr(inst, field_name)

        field_value = _to_inst(msg[field_name], field_ptype,
                    roottype, field_inst, field_stack)

        setattr(inst, field_name, field_value)

    return inst
