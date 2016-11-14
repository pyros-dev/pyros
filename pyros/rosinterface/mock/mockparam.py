from __future__ import absolute_import, print_function

from ...baseinterface import TransientIf


class MockParam(TransientIf):
    """
    MockParam is the class handling mock behavior for Param
    """
    def __init__(self, param_name, param_type):

        # getting the fullname to make sure we start with /
        param_name = param_name if param_name.startswith('/') else '/' + param_name

        param_type = param_type()  # or None ??

        super(MockParam, self).__init__(param_name, param_type)

    def cleanup(self):
        pass

    def setval(self, val):
        self.value = val
        return True

    def getval(self):
        return self.value
