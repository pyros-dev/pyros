import os
import sys
import setuptools

# Ref : https://packaging.python.org/single_source_version/#single-sourcing-the-version
with open('pyros/_version.py') as vf:
    exec(vf.read())

# Best Flow :
# git changelog >CHANGELOG.rst
# git commit "updating changelog"
# change version in code and changelog
# git commit "v<M.m.p>"
# python setup.py publish
# python setup.py tag
# => try to do a simple "release" command


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PublishCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "releases pyros_setup to Pypi"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        os.system("python setup.py sdist")
        os.system("python setup.py bdist_wheel")
        # OLD way:
        # os.system("python setup.py sdist bdist_wheel upload")
        # NEW way:
        # Ref: https://packaging.python.org/distributing/
        os.system("twine upload dist/*")
        print("You probably want to also tag the version now:")
        print("  python setup.py tag")
        sys.exit()


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class TagCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "tag a release of pyros_setup"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        os.system("git tag -a {0} -m 'version {0}'".format(__version__))
        os.system("git push --tags")
        sys.exit()

setuptools.setup(name='pyros',
    version=__version__,
    description='ROS Node to provide ROS introspection for non-ROS users.',
    url='http://github.com/asmodehn/pyros',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
    packages=[
        'pyros',
        'pyros.tests',
        'pyros.baseinterface',
        'pyros.mockinterface',
        'pyros.mockinterface.tests',
        'pyros.rosinterface',
        'pyros.rosinterface.tests',
        'pyros.rosinterface.rostests',
        'pyros.zmpinterface',
        'pyros.zmpinterface.tests',
    ],
    entry_points={
        'console_scripts': [
            'pyros = pyros.__main__:nosemain'
        ]
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'tblib',  # this might not always install six (latest version does not)
        'six',
        'pyzmq',
        'pyzmp==0.0.14',  # lets be rigorous since we are working on both at the same time...
        'pyros_setup>=0.1.2',  # Careful : pyros-setup < 0.0.8 might already be installed as a deb in /opt/ros/indigo/lib/python2.7/dist-packages/
        'pyros_config>=0.1.4',
        'nose>=1.3.7',
        'mock==1.0.1',  # old mock to be compatible with trusty versions
    ],
    # Reference for optional dependencies : http://stackoverflow.com/questions/4796936/does-pip-handle-extras-requires-from-setuptools-distribute-based-sources
    test_suite="nose.collector",
    tests_require=["nose"],
    zip_safe=False,  # TODO testing...
)
