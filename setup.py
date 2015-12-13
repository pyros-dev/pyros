# This setup is usable by catkin, or on its own as usual python setup.py

_CATKIN = False
try:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup
    _CATKIN = True
except Exception, e:
    from setuptools import setup

# CAREFUL distutils and setuptools take different arguments and have different behaviors
# ROS PACKAGING
if _CATKIN:  # using distutils : https://docs.python.org/2/distutils
    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=[
            'zmp',
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
            'tblib',
        ],
        package_dir={
            'zmp': 'src/zmp',
            'pyros': 'src/pyros',
            'tblib': 'deps/python-tblib/src/tblib',
        },
        py_modules=[
            'six',
        ],
    )
    setup(**setup_args)
# PYTHON PACKAGING
else:  # using setuptools : http://pythonhosted.org/setuptools/

    setup(name='pyros',
        version='0.1.0',
        description='ROS Node to provide ROS introspection for non-ROS users.',
        url='http://github.com/asmodehn/rostful-node',
        author='AlexV',
        author_email='asmodehn@gmail.com',
        license='BSD',
        packages=[
            'pyros',
            'mockinterface',
            'rosinterface',
            'roconinterface',
        ],
        package_dir={'': 'src'},
        # this is better than using package data ( since behavior is a bit different from distutils... )
        include_package_data=True,  # use MANIFEST.in during install.
        install_requires=[
            'tblib',  # this might not always install six (latest version does not)
            'six',
            #'dill',
            'funcsigs',
        ],
        zip_safe=False,  # TODO testing...
    )
