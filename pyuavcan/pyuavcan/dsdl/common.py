#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import os

class DsdlException(Exception):
    def __init__(self, text, file=None, line=None):
        Exception.__init__(self, text)
        self.file = file
        self.line = line

    def __str__(self):
        if self.file and self.line:
            return '%s:%d: %s' % (pretty_filename(self.file), self.line, Exception.__str__(self))
        if self.file:
            return '%s: %s' % (pretty_filename(self.file), Exception.__str__(self))
        return Exception.__str__(self)


def pretty_filename(filename):
    a = os.path.abspath(filename)
    r = os.path.relpath(filename)
    return a if len(a) < len(r) else r
