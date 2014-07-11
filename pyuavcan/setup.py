#!/usr/bin/env python

from distutils.core import setup

args = dict(
    name='pyuavcan',
    version='0.1',
    description='UAVCAN for Python',
    packages=['pyuavcan', 'pyuavcan.dsdl'],
    author='Pavel Kirienko',
    author_email='pavel.kirienko@gmail.com',
    url='http://uavcan.org',
    license='MIT'
)

setup(**args)
