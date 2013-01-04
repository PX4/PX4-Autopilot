#
# minixsv, Release 0.9.0
# file: __init__.py
#
# minixsv package file
#
# history:
# 2004-10-26 rl   created
#
# Copyright (c) 2004-2008 by Roland Leuthe.  All rights reserved.
#
# --------------------------------------------------------------------
# The minixsv XML schema validator is
#
# Copyright (c) 2004-2008 by Roland Leuthe
#
# By obtaining, using, and/or copying this software and/or its
# associated documentation, you agree that you have read, understood,
# and will comply with the following terms and conditions:
#
# Permission to use, copy, modify, and distribute this software and
# its associated documentation for any purpose and without fee is
# hereby granted, provided that the above copyright notice appears in
# all copies, and that both that copyright notice and this permission
# notice appear in supporting documentation, and that the name of
# the author not be used in advertising or publicity
# pertaining to distribution of the software without specific, written
# prior permission.
#
# THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD
# TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANT-
# ABILITY AND FITNESS.  IN NO EVENT SHALL THE AUTHOR
# BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY
# DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
# WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
# ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
# OF THIS SOFTWARE.
# --------------------------------------------------------------------


######################################################################
# PUBLIC DEFINITIONS
######################################################################


# supported XML interfaces

XMLIF_MINIDOM     = "XMLIF_MINIDOM"
XMLIF_4DOM        = "XMLIF_4DOM"
XMLIF_ELEMENTTREE = "XMLIF_ELEMENTTREE"



# namespace definitions

EMPTY_PREFIX    = None

EMPTY_NAMESPACE = None
XML_NAMESPACE   = "http://www.w3.org/XML/1998/namespace"
XMLNS_NAMESPACE = "http://www.w3.org/2000/xmlns/"
XSD_NAMESPACE   = "http://www.w3.org/2001/XMLSchema"
XSI_NAMESPACE   = "http://www.w3.org/2001/XMLSchema-instance"


# definition of minixsv path 

import os
MINIXSV_DIR = os.path.dirname(__file__)


# error handling definitions

from xsvalErrorHandler import IGNORE_WARNINGS, PRINT_WARNINGS, STOP_ON_WARNINGS
from xsvalErrorHandler import XsvalError

