import sys
import os

# logging configuration should be here to not be imported by python users of pyros.
# only used from command line

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

if __name__ == "__main__":
    nosemain()
