#
# genxmlif, Release 0.9.0
# file: __init__.py
#
# genxmlif package file
#
# history:
# 2005-04-25 rl   created
#
# Copyright (c) 2005-2008 by Roland Leuthe.  All rights reserved.
#
# --------------------------------------------------------------------
# The generic XML interface is
#
# Copyright (c) 2005-2008 by Roland Leuthe
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

XINC_NAMESPACE  = "http://www.w3.org/2001/XInclude"


# definition of genxmlif path 

import os
GENXMLIF_DIR = os.path.dirname(__file__)


########################################
# central function to choose the XML interface to be used
#

def chooseXmlIf (xmlIf, verbose=0, useCaching=1, processXInclude=1):
    if xmlIf == XMLIF_MINIDOM:
        import xmlifMinidom
        return xmlifMinidom.XmlInterfaceMinidom(verbose, useCaching, processXInclude)

    elif xmlIf == XMLIF_4DOM:
        import xmlif4Dom
        return xmlif4Dom.XmlInterface4Dom(verbose, useCaching, processXInclude)

    elif xmlIf == XMLIF_ELEMENTTREE:
        import xmlifElementTree
        return xmlifElementTree.XmlInterfaceElementTree(verbose, useCaching, processXInclude)

    else:
        raise AttributeError, "Unknown XML interface: %s" %(xmlIf)


########################################
# define own exception for GenXmlIf errors
# The following errors/exceptions are mapped to a GenxmlIf exception:
# - Expat errors
# - XInclude errors
#
class GenXmlIfError (StandardError):
    pass

