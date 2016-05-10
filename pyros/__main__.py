import sys
import os
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
