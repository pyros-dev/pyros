from setuptools import setup

setup(name='pyros',
    version='0.1.0',
    description='ROS Node to provide ROS introspection for non-ROS users.',
    url='http://github.com/asmodehn/pyros',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
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
    ],
    package_dir={
        'zmp': 'src/zmp',
        'pyros': 'src/pyros',
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'tblib',  # this might not always install six (latest version does not)
        'six',
        'pyzmq',
        'pyros_setup',
    ],
    zip_safe=False,  # TODO testing...
)
