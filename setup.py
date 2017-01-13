import os
import subprocess
import sys
import setuptools

# Ref : https://packaging.python.org/single_source_version/#single-sourcing-the-version
with open('pyros/_version.py') as vf:
    exec(vf.read())

# Best Flow :
# Clean previous build & dist
# $ gitchangelog >CHANGELOG.rst
# change version in code and changelog
# $ python setup.py prepare_release
# WAIT FOR TRAVIS CHECKS
# $ python setup.py publish
# => TODO : try to do a simpler "release" command

# TODO : command to retrieve extra ROS stuff from a third party release repo ( for ROS devs ). useful in dev only so maybe "rosdevelop" ? or via catkin_pip ?
# TODO : command to release to Pip and ROS (bloom) same version one after the other...


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PrepareReleaseCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "prepare a release of pyros"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        # TODO :
        # $ gitchangelog >CHANGELOG.rst
        # $ git commit CHANGELOG.rst -m "updating changelog"
        # change version in code and changelog
        subprocess.check_call("git commit CHANGELOG.rst pyros/_version.py package.xml -m 'v{0}'".format(__version__), shell=True)
        subprocess.check_call("git push", shell=True)

        print("You should verify travis checks, and you can publish this release with :")
        print("  python setup.py publish")
        sys.exit()


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PublishCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "releases pyros to Pypi"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""
        # TODO : clean build/ and dist/ before building...
        subprocess.check_call("python setup.py sdist", shell=True)
        subprocess.check_call("python setup.py bdist_wheel", shell=True)
        # OLD way:
        # os.system("python setup.py sdist bdist_wheel upload")
        # NEW way:
        # Ref: https://packaging.python.org/distributing/
        subprocess.check_call("twine upload dist/*", shell=True)

        subprocess.check_call("git tag -a {0} -m 'version {0}'".format(__version__), shell=True)
        subprocess.check_call("git push --tags", shell=True)
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
        'pyros.rosinterface',
        'pyros.rosinterface.api',
        'pyros.rosinterface.mock',
        'pyros.rosinterface.mock.tests',
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
        'pyzmp>=0.0.14',  # lets match the requirement in package.xml (greater than)
        'pyros_setup>=0.1.5',  # Careful : pyros-setup < 0.0.8 might already be installed as a deb in /opt/ros/indigo/lib/python2.7/dist-packages/
        'pyros_config>=0.1.4',
        'nose>=1.3.7',
        'mock>=1.0.1',  # old mock to be compatible with trusty versions
    ],
    # Reference for optional dependencies : http://stackoverflow.com/questions/4796936/does-pip-handle-extras-requires-from-setuptools-distribute-based-sources
    test_suite="nose.collector",
    tests_require=["nose"],
    cmdclass={
        'prepare_release': PrepareReleaseCommand,
        'publish': PublishCommand,
    },
    zip_safe=False,  # TODO testing...
)
