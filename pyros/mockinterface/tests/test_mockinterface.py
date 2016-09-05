from __future__ import absolute_import
from __future__ import print_function

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from pyros.mockinterface import MockInterface, mock_service
from pyros.mockinterface.mockservice import statusecho_service, MockService

import nose
from nose.tools import timed, assert_true, assert_false, assert_equal, assert_raises


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)  # this doesnt break if  hanging forever. need to replace with a breaking version
def test_mockinterface_update_services_c1():
    svc_name = '/awesome_service'
    mockif = MockInterface()

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed yet
    diffupdate = mockif.update_services(add_names=[svc_name], remove_names=[])
    assert_false(diffupdate.added)  # service not detected cannot be added
    assert_false(diffupdate.removed)

    diffupdate = mockif.update_services(add_names=[], remove_names=[svc_name])
    assert_false(diffupdate.added)
    assert_false(diffupdate.removed)  # service not added cannot be removed

    with mock_service(svc_name, statusecho_service):  # simulating service appearing
        mockif.services_args.add(svc_name)  # adding it to regex list to allow it to be exposed

        diffupdate = mockif.update_services(add_names=[svc_name], remove_names=[])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # service exposed can be added

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed now
        assert_true(isinstance(svc, MockService))  # service type is MockService

        diffupdate = mockif.update_services(add_names=[], remove_names=[svc_name])
        assert_false(diffupdate.added)
        assert_true(diffupdate.removed)  # service exposed can be deleted

        assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mockinterface_update_services_c2():
    svc_name = '/awesome_service'
    mockif = MockInterface()

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed yet
    diffupdate = mockif.update_services(add_names=[svc_name], remove_names=[])
    assert_false(diffupdate.added)  # service not detected cannot be added
    assert_false(diffupdate.removed)

    diffupdate = mockif.update_services(add_names=[], remove_names=[svc_name])
    assert_false(diffupdate.added)
    assert_false(diffupdate.removed)  # service not added cannot be removed

    with mock_service(svc_name, statusecho_service):  # simulating service appearing
        mockif.services_args.add(svc_name)  # adding it to regex list to allow it to be exposed

        diffupdate = mockif.update_services(add_names=[svc_name], remove_names=[])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # service exposed can be added

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed now
        assert_true(isinstance(svc, MockService))  # service type is MockService

    svc = mockif.services.get(svc_name)
    assert_true(svc is not None)  # service is still exposed even though it s gone from the system we interface to
    # WARNING : Using the service in this state will trigger errors.
    # These should be handled by the service class.
    # TODO : assert this

    diffupdate = mockif.update_services(add_names=[], remove_names=[svc_name])
    assert_false(diffupdate.added)
    assert_true(diffupdate.removed)  # service non available (but added) can be deleted

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mockinterface_expose_update_services_fullname():
    svc_name = '/awesome_service'
    mockif = MockInterface()

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed yet
    diffupdate = mockif.expose_services([svc_name])
    assert_false(diffupdate.added)  # service not detected cannot be added
    assert_false(diffupdate.removed)

    with mock_service(svc_name, statusecho_service):

        diffupdate = mockif.expose_services([svc_name])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # service available can be detected and be added

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed now
        assert_true(isinstance(svc, MockService))  # service type is MockService

    svc = mockif.services.get(svc_name)
    assert_true(svc is not None)  # service exposed now
    assert_true(isinstance(svc, MockService))  # service type is MockService

    # WARNING : Using the service in this state will trigger errors.
    # These should be handled by the service class.
    # TODO : assert this

    diffupdate = mockif.services_change_detect()
    assert_false(diffupdate.added)
    assert_true(diffupdate.removed)  # service lost can be detected and be removed

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore

    diffupdate = mockif.expose_services([svc_name])
    assert_false(diffupdate.removed)
    assert_false(diffupdate.added)  # new expose call doesn't change anything

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mockinterface_update_expose_services_fullname():
    svc_name = '/awesome_service'
    mockif = MockInterface()

    diffupdate = mockif.services_change_detect()
    assert_false(diffupdate.added)  # service not available is not detected and not added
    assert_false(diffupdate.removed)  # service not added previously is not removed

    with mock_service(svc_name, statusecho_service):

        diffupdate = mockif.services_change_detect()
        assert_false(diffupdate.removed)
        assert_false(diffupdate.added)  # service available is not detected and added without previous expose call

        svc = mockif.services.get(svc_name)
        assert_true(svc is None)  # service not exposed now

        diffupdate = mockif.expose_services([svc_name])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # new expose call add the service because it is already available

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed now
        assert_true(isinstance(svc, MockService))  # service type is MockService

        diffupdate = mockif.services_change_detect()
        assert_false(diffupdate.removed)
        assert_false(diffupdate.added)  # new detection call doesnt change anything

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service still exposed
        assert_true(isinstance(svc, MockService))  # service type is still MockService

        diffupdate = mockif.expose_services([])
        assert_false(diffupdate.added)
        assert_true(diffupdate.removed)  # new expose call can remove the service even if it is still available

        assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore

        diffupdate = mockif.expose_services([svc_name])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # new expose call can readd if it is still available

    svc = mockif.services.get(svc_name)
    assert_true(svc is not None)  # service exposed now
    assert_true(isinstance(svc, MockService))  # service type is MockService

    # WARNING : Using the service in this state will trigger errors.
    # These should be handled by the service class.
    # TODO : assert this

    diffupdate = mockif.expose_services([])
    assert_false(diffupdate.added)
    assert_true(diffupdate.removed)  # new expose call can remove the service even if it is not available

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore

    diffupdate = mockif.services_change_detect()
    assert_false(diffupdate.added)  # no appeared service : nothing is added
    assert_false(diffupdate.removed)  # disappeared service was already removed

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mockinterface_expose_services_regex():
    svc_name = '/awesome_service'
    svc_regex = '/.*'
    mockif = MockInterface()

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed yet
    diffupdate = mockif.expose_services([svc_regex])
    assert_false(diffupdate.added)  # service not detected cannot be added
    assert_false(diffupdate.removed)

    with mock_service(svc_name, statusecho_service):

        diffupdate = mockif.services_change_detect()
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # new detection call finds the service and adds it

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed
        assert_true(isinstance(svc, MockService))  # service type is MockService

    diffupdate = mockif.services_change_detect()
    assert_false(diffupdate.added)
    assert_true(diffupdate.removed)  # new detection call finds the service and removes it

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mockinterface_update_expose_services_fullname_diff():
    svc_name = '/awesome_service'
    mockif = MockInterface()

    diffupdate = mockif.services_change_diff([], [])
    assert_false(diffupdate.added)  # service not passed in diff is not detected and not added
    assert_false(diffupdate.removed)  # service not added previously is not removed

    with mock_service(svc_name, statusecho_service):

        diffupdate = mockif.services_change_diff([svc_name], [])
        assert_false(diffupdate.removed)
        assert_false(diffupdate.added)  # service available is not detected and added without previous expose call

        svc = mockif.services.get(svc_name)
        assert_true(svc is None)  # service not exposed now

        diffupdate = mockif.expose_services([svc_name])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # new expose call add the service because it is already available

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed now
        assert_true(isinstance(svc, MockService))  # service type is MockService

        diffupdate = mockif.services_change_diff([], [])
        assert_false(diffupdate.removed)
        assert_false(diffupdate.added)  # empty diff call doesnt change anything

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service still exposed
        assert_true(isinstance(svc, MockService))  # service type is still MockService

        diffupdate = mockif.expose_services([])
        assert_false(diffupdate.added)
        assert_true(diffupdate.removed)  # new expose call can remove the service even if it is still available

        assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore

        diffupdate = mockif.expose_services([svc_name])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # new expose call can readd if it is still available

    svc = mockif.services.get(svc_name)
    assert_true(svc is not None)  # service exposed now
    assert_true(isinstance(svc, MockService))  # service type is MockService

    # WARNING : Using the service in this state will trigger errors.
    # These should be handled by the service class.
    # TODO : assert this

    diffupdate = mockif.expose_services([])
    assert_false(diffupdate.added)
    assert_true(diffupdate.removed)  # new expose call can remove the service even if it is not available

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore

    diffupdate = mockif.services_change_diff([], [])
    assert_false(diffupdate.added)  # no service passed in diff : nothing is added
    assert_false(diffupdate.removed)  # disappeared service was already removed

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore


# @nose.SkipTest  # to help debugging ( FIXME : how to programmatically start only one test - maybe in fixture - ? )
@timed(5)
def test_mockinterface_expose_services_regex_diff():
    svc_name = '/awesome_service'
    svc_regex = '/.*'
    mockif = MockInterface()

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed yet
    diffupdate = mockif.expose_services([svc_regex])
    assert_false(diffupdate.added)  # service not detected cannot be added
    assert_false(diffupdate.removed)

    with mock_service(svc_name, statusecho_service):

        diffupdate = mockif.services_change_diff([svc_name], [])
        assert_false(diffupdate.removed)
        assert_true(diffupdate.added)  # new diff call finds the service and adds it

        svc = mockif.services.get(svc_name)
        assert_true(svc is not None)  # service exposed
        assert_true(isinstance(svc, MockService))  # service type is MockService

    diffupdate = mockif.services_change_diff([], [svc_name])
    assert_false(diffupdate.added)
    assert_true(diffupdate.removed)  # new diff call finds the service and removes it

    assert_true(mockif.services.get(svc_name) is None)  # service not exposed anymore

#TODO : test exception raised properly when update transient cannot happen


# TODO : Same for topics and params

if __name__ == '__main__':

    import nose
    nose.runmodule()




