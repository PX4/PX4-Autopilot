#!/usr/bin/env python3

from distutils.core import setup

args = dict(
    name='uavcan_dsdlc',
    version='0.1',
    description='UAVCAN DSDL compiler for libuavcan',
    packages=['libuavcan_dsdl_compiler'],
    package_data={'libuavcan_dsdl_compiler': ['data_type_template.tmpl']},
    scripts=['uavcan_dsdlc'],
    requires=['mako', 'pyuavcan'],
    author='Pavel Kirienko',
    author_email='pavel.kirienko@gmail.com',
    url='http://uavcan.org',
    license='MIT'
)

setup(**args)
