#!/usr/bin/python
# All ways to run pyros and all Manager commands
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys

import six

import click

# logging configuration should be here to not be imported by python users of pyros.
# only used from command line

import logging
import logging.config
# Setting up logging for this test
logging.config.dictConfig(
    {
        'version': 1,
        'formatters': {
            'verbose': {
                'format': '%(levelname)s %(asctime)s %(module)s %(process)d %(thread)d %(message)s'
            },
            'simple': {
                'format': '%(levelname)s %(name)s:%(message)s'
            },
        },
        'handlers': {
            'null': {
                'level': 'DEBUG',
                'class': 'logging.NullHandler',
            },
            'console': {
                'level': 'DEBUG',
                'class': 'logging.StreamHandler',
                'formatter': 'simple'
            },
        },
        'loggers': {
            'pyros_config': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup': {
                'handlers': ['console'],
                'level': 'INFO',
            },
            'pyros': {
                'handlers': ['console'],
                'level': 'INFO',
            }
        }
    }
)


import nose
import pkg_resources

_path = pkg_resources.resource_filename("pyros", "__main__.py")
_parent = os.path.normpath(os.path.join(os.path.dirname(_path), ".."))


# importing current package if needed ( solving relative package import from __main__ problem )
# if __package__ is None:
#     sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
#     from rostful import create_app, set_pyros_client, setup_app_routes
# else:
#     from . import create_app, set_pyros_client, setup_app_routes

#TODO : enable starting different interface nodes for different MP frameworks


def nosemain():
    args = sys.argv + [opt for opt in (
        # "--exe",  # DO NOT look in exe (maybe old rostests ?)
        # "--all-modules",  # DO NOT look in all modules
        "--traverse-namespace",
        "--verbosity=2",
        "--with-id",
        "--with-xunit",
        "--where=%s" % _parent
    )
    if opt not in sys.argv
    ]
    nose.run(argv=args)


def pyros_rosinterface_launch(node_name=None, pyros_config=None, ros_argv=None):

    node_name = node_name or 'pyros_rosinterface'
    ros_argv = ros_argv or []

    # dynamic setup and import
    try:
        import pyros
        node_proc = pyros.PyrosROS(
            node_name,
            ros_argv
        )
        # Attribute error is triggered when using pyros < 0.1.0
    except (ImportError, AttributeError) as e:
        #logging.warn("{name} Error: Could not import pyros : {e}".format(name=__name__, e=e))
        ### TMP ###
        logging.warn("{name} Attempting bwcompat import : {e}".format(name=__name__, e=e))
        # BWcompat with old pyros :
        try:
            # this will import rosinterface and if needed simulate ROS setup
            import sys
            import pyros
            import pyros.rosinterface
            # this doesnt work with celery handling of imports
            # sys.modules["pyros.rosinterface"] = pyros.rosinterface.delayed_import_auto(
            #     distro='indigo',
            #     base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
            # )
            pyros.rosinterface = pyros.rosinterface.delayed_import_auto(
                distro='indigo',
                base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
            )

            node_proc = pyros.rosinterface.PyrosROS(
                node_name,
                ros_argv,
                base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')
            )

            node_proc.configure(pyros_config)

        except ImportError as e:
            logging.warn("{name} Error: Could not import pyros.rosinterface : {e}".format(name=__name__, e=e))
            raise

    return node_proc


# Change that into something cleaner and related with the app itself (not server)
# Middleware ? app context ? Tricky with dynamic imports...
# middleware ref : https://github.com/miguelgrinberg/python-engineio/blob/master/engineio/middleware.py#L49
def pyros_start(config, ros_args='', pyros_ctx_impl=None):

    # implementing Config.get_namespace() for flask version < 1.0:
    namespace = 'PYROS_'
    trim_namespace = True
    lowercase = False
    rv = {}
    for k, v in six.iteritems(config):
        if not k.startswith(namespace):
            continue
        if trim_namespace:
            key = k[len(namespace):]
        else:
            key = k
        if lowercase:
            key = key.lower()
        rv[key] = v
    # rv should now contain a dictionary of namespaced key:value from config

    try:
        from pyros import pyros_ctx, PyrosClient
    except Exception as e:
        logging.error("pyros module is not accessible in sys.path. It is required to run rostful.", exc_info=True)
        logging.error("sys.path = {0}".format(sys.path))
        raise

    # default to real module, if no other implementation passed as parameter (used for mock)
    pyros_ctx_impl = pyros_ctx_impl or pyros_ctx

    # One PyrosNode is needed for Flask.
    # TODO : check everything works as expected, even if the WSGI app is used by multiple processes

    try:
        node_ctx_gen = pyros_ctx_impl(name='rostful', argv=ros_args, pyros_config=rv)
    except TypeError as te:
        # bwcompat trick
        node_ctx_gen = pyros_ctx_impl(name='rostful', argv=ros_args,
                            base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    return node_ctx_gen


# TODO : handle ros arguments here
# http://click.pocoo.org/5/commands/#group-invocation-without-command
@click.group()
def cli():
    pass


@cli.command()
def test():
    errno = nosemain()
    sys.exit(errno)


#
# Arguments' default value is None here
# to use default values from config file if one is provided.
# If no config file is provided, internal defaults are used.
#
@cli.command()
@click.option('-i', '--interface', default='ros', type=click.Choice(['ros', 'ros_mock']))
# @click.option('-a', '--async', default=False)  # wether to activate async implementation (instead of pyros main loop)
@click.option('-c', '--config', default=None)  # this is the last possible config override, and has to be explicit.
@click.option('-l', '--logfile', default=None)  # this is the last possible logfile override, and has to be explicit.
@click.option('ros_args', '-r', '--ros-arg', multiple=True, default='')
def run(interface, config, logfile, ros_args):
    """
    Start a pyros node.
    :param interface: the interface implementation (ROS, Mock, ZMP, etc.)
    :param config: the config file path, absolute, or relative to working directory
    :param logfile: the logfile path, absolute, or relative to working directory
    :param ros_args: the ros arguments (useful to absorb additional args when launched with roslaunch)
    """
    logging.info(
        'pyros started with : interface {interface} config {config} logfile {logfile} ros_args {ros_args}'.format(
            interface=interface, config=config, logfile=logfile, ros_args=ros_args))

    if interface == 'ros':
        node_proc = pyros_rosinterface_launch(node_name='pyros_rosinterface', pyros_config=config, ros_argv=ros_args)
    else:
        node_proc = None  # NOT IMPLEMENTED

    # node_proc.daemon = True  # we do NOT want a daemon(would stop when this main process exits...)
    client_conn = node_proc.start()  # in a sub process
    # DISABLING THIS FOR NOW, process tree is a bit unexpected...
    # TODO : investigate
    # client_conn = node_proc.run()  # in same process


if __name__ == '__main__':
   cli()
