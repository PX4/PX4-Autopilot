#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

import os

class DsdlException(Exception):
    def __init__(self, text, file=None, line=None):
        super().__init__(text)
        self.file = file
        self.line = line

    def __str__(self):
        if self.file and self.line:
            return '%s:%s: %s' % (pretty_filename(self.file), self.line, super().__str__())
        if self.file:
            return '%s: %s' % (pretty_filename(self.file), super().__str__())
        return super().__str__()


def pretty_filename(filename):
    a = os.path.abspath(filename)
    r = os.path.relpath(filename)
    return a if len(a) < len(r) else r
