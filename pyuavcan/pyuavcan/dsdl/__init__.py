#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

'''
This module implements a fully compliant UAVCAN DSDL parser.
Please read the specs at http://uavcan.org.
'''

from .parser import Parser, parse_namespaces, \
                    Type, PrimitiveType, ArrayType, CompoundType, \
                    Attribute, Field, Constant

from .common import DsdlException

__all__ = ['Parser', 'parse_namespaces',
           'Type', 'PrimitiveType', 'ArrayType', 'CompoundType',
           'Attribute', 'Field', 'Constant',
           'DsdlException']
