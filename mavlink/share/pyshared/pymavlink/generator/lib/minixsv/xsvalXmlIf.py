#
# minixsv, Release 0.9.0
# file: xsvalXmlIf.py
#
# XML interface classes (derived wrapper classes)
#
# history:
# 2007-07-04 rl   created
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


from types import TupleType
from ..genxmlif.xmlifApi   import XmlElementWrapper


class XsvXmlElementWrapper (XmlElementWrapper):
    
#++++++++++++ special XSD validation support ++++++++++++++++++++
    def __init__(self, element, treeWrapper, curNs=[], initAttrSeq=1):
        XmlElementWrapper.__init__(self, element, treeWrapper, curNs, initAttrSeq)
        self.schemaRootNode = None
        self.xsdNode = None
        self.xsdAttrNodes = {}

    
    def cloneNode(self, deep, cloneCallback=None):
        return XmlElementWrapper.cloneNode(self, deep, self.cloneCallback)


    def cloneCallback(self, nodeCopy):
        nodeCopy.schemaRootNode = self.schemaRootNode
        nodeCopy.xsdNode = self.xsdNode
        nodeCopy.xsdAttrNodes = self.xsdAttrNodes.copy()


    def getSchemaRootNode (self):
        """Retrieve XML schema root node which this element node belongs to
           (e.g. for accessing target namespace attribute).
        
        Returns XML schema root node which this element node belongs to
        """
        return self.schemaRootNode
    

    def setSchemaRootNode (self, schemaRootNode):
        """Store XML schema root node which this element node belongs to.
        
        Input parameter:
            schemaRootNode:    schema root node which this element node belongs to
        """
        self.schemaRootNode = schemaRootNode


    def getXsdNode (self):
        """Retrieve XML schema node responsible for this element node.
        
        Returns XML schema node responsible for this element node.
        """
        return self.xsdNode
    

    def setXsdNode (self, xsdNode):
        """Store XML schema node responsible for this element node.
        
        Input parameter:
            xsdNode:    responsible XML schema ElementWrapper
        """
        self.xsdNode = xsdNode


    def getXsdAttrNode (self, tupleOrAttrName):
        """Retrieve XML schema node responsible for the requested attribute.

        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
        Returns XML schema node responsible for the requested attribute.
        """
        try:
            return self.xsdAttrNodes[tupleOrAttrName]
        except:
            if isinstance(tupleOrAttrName, TupleType):
                if tupleOrAttrName[1] == '*' and len(self.xsdAttrNodes) == 1:
                    return self.xsdAttrNodes.values()[0]
            return None
    

    def setXsdAttrNode (self, tupleOrAttrName, xsdAttrNode):
        """Store XML schema node responsible for the given attribute.
        
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
            xsdAttrNode:      responsible XML schema ElementWrapper
        """
        self.xsdAttrNodes[tupleOrAttrName] = xsdAttrNode


