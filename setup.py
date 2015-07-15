# This setup is usable by catkin, or on its own as usual python setup.py

_CATKIN = False
try:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup
    _CATKIN = True
except Exception, e:
    from setuptools import setup

# CAREFUL distutils and setuptools take different argument sand have different behaviors
if _CATKIN:  # using distutils : https://docs.python.org/2/distutils
    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=['rostful_node', 'rosinterface', 'roconinterface'],
        package_dir={'': 'src'},
    )
    setup(**setup_args)

else:  # using setuptools : http://pythonhosted.org/setuptools/

    setup(name='rostful_node',
        version='0.0.6',
        description='ROS Node to provide ROS introspection for non-ROS users.',
        url='http://github.com/asmodehn/rostful-node',
        author='AlexV',
        author_email='asmodehn@gmail.com',
        license='BSD',
        packages=['rostful_node', 'rosinterface', 'roconinterface'],
        package_dir={'': 'src'},
        # this is better than using package data ( since behavior is a bit different from distutils... )
        include_package_data=True,  # use MANIFEST.in during install.
        install_requires=[
        ],
        zip_safe=False,  # TODO testing...
    )
