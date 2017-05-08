import os
import shutil
import subprocess
import sys
import tempfile
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

        # change version in code and changelog before running this
        subprocess.check_call("git commit CHANGELOG.rst pyros/_version.py CHANGELOG.rst -m 'v{0}'".format(__version__), shell=True)
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


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class RosDevelopCommand(setuptools.Command):

    """Command to mutate this package to a ROS package, using its ROS release repository"""
    description = "mutate this package to a ROS package using its release repository"
    user_options = []

    def initialize_options(self):
        """init options"""
        # TODO : add distro selector
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        # dynamic import for this command only to not need these in usual python case...
        import git
        import yaml

        """runner"""
        repo_path = tempfile.mkdtemp(prefix='rosdevelop-' + os.path.dirname(__file__))  # TODO get actual package name ?
        print("Getting ROS release repo in {0}...".format(repo_path))
        # TODO : get release repo from ROSdistro
        rosrelease_repo = git.Repo.clone_from('https://github.com/asmodehn/pyros-rosrelease.git', repo_path)

        # Reset our working tree to master
        origin = rosrelease_repo.remotes.origin
        rosrelease_repo.remotes.origin.fetch()  # assure we actually have data. fetch() returns useful information
        # Setup a local tracking branch of a remote branch
        rosrelease_repo.create_head('master', origin.refs.master).set_tracking_branch(origin.refs.master).checkout()

        print("Reading tracks.yaml...")
        with open(os.path.join(rosrelease_repo.working_tree_dir, 'tracks.yaml'), 'r') as tracks:
            try:
                tracks_dict = yaml.load(tracks)
            except yaml.YAMLError as exc:
                raise

        patch_dir = tracks_dict.get('tracks', {}).get('indigo', {}).get('patches', {})

        print("Found patches for indigo in {0}".format(patch_dir))
        src_files = os.listdir(os.path.join(rosrelease_repo.working_tree_dir, patch_dir))

        working_repo = git.Repo(os.path.dirname(os.path.abspath(__file__)))

        # adding patched files to ignore list if needed (to prevent accidental commit of patch)
        # => BETTER if the patch do not erase previous file. TODO : fix problem with both .travis.yml
        with open(os.path.join(working_repo.working_tree_dir, '.gitignore'), 'a+') as gitignore:
            skipit = []
            for line in gitignore:
                if line in src_files:
                    skipit += line
                else:  # not found, we are at the eof
                    for f in src_files:
                        if f not in skipit:
                            gitignore.write(f+'\n')  # append missing data

        working_repo.git.add(['.gitignore'])  # adding .gitignore to the index so git applies it (and hide new files)

        for file_name in src_files:
            print("Patching {0}".format(file_name))
            full_file_name = os.path.join(rosrelease_repo.working_tree_dir, patch_dir, file_name)
            if os.path.isfile(full_file_name):
                # Special case for package.xml and version template string
                if file_name == 'package.xml':
                    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'package.xml'), "wt") as fout:
                        with open(full_file_name, "rt") as fin:
                            for line in fin:
                                fout.write(line.replace(':{version}', __version__))  # TODO: proper template engine ?
                else:
                    shutil.copy(full_file_name, os.path.dirname(os.path.abspath(__file__)))

        sys.exit()


class ROSPublishCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "releases pyros to ROS"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""
        # TODO : distro from parameter. default : ['indigo', 'jade', 'kinetic']
        subprocess.check_call("git tag -a ros-{0} -m 'version {0} for ROS'".format(__version__), shell=True)
        subprocess.check_call("git push --tags", shell=True)

        subprocess.check_call("bloom-release --rosdistro indigo --track indigo pyros", shell=True)
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
        'pyros.client',
        'pyros.client.tests',
        'pyros.server',
        'pyros.server.tests',
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
        'pyros_setup>=0.1.5',  # Careful : pyros-setup < 0.0.8 might already be installed as a deb in /opt/ros/indigo/lib/python2.7/dist-packages/ => we still need to force hte install in the venv to have permissions to create hte configuration file...
        'pyros_config>=0.1.4',
        'pyros-common',
        'nose>=1.3.7',
        'mock>=1.0.1',  # old mock to be compatible with trusty versions
    ],
    extras_require={
      'ros': 'pyros_interfaces_ros',
    },
    dependency_links=[
        'git+https://github.com/asmodehn/pyros-rosinterface.git@namespace#egg=pyros_interfaces_ros'
    ],
    # Reference for optional dependencies : http://stackoverflow.com/questions/4796936/does-pip-handle-extras-requires-from-setuptools-distribute-based-sources

    test_suite="nose.collector",
    tests_require=["nose"],
    cmdclass={
        'rosdevelop': RosDevelopCommand,
        'prepare_release': PrepareReleaseCommand,
        'publish': PublishCommand,
        'rospublish': ROSPublishCommand,
    },
    zip_safe=False,  # TODO testing...
)
