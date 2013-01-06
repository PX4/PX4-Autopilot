#
# genxmlif, Release 0.9.0
# file: xmlifbase.py
#
# XML interface base classes
#
# history:
# 2005-04-25 rl   created
# 2006-08-18 rl   some methods for XML schema validation support added
# 2007-05-25 rl   performance optimization (caching) added, bugfixes for XPath handling
# 2007-07-04 rl   complete re-design, API classes moved to xmlifApi.py
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

__author__  = "Roland Leuthe <roland@leuthe-net.de>"
__date__    = "28 July 2008"
__version__ = "0.9"

from xml.dom    import XML_NAMESPACE, XMLNS_NAMESPACE
from xmlifUtils import NsNameTupleFactory, convertToAbsUrl



########################################
# XmlIf builder extension base class
# All not implemented methods have to be overloaded by the derived class!!
#

class XmlIfBuilderExtensionBase:
    """XmlIf builder extension base class.
    
    This class provides additional data (e.g. line numbers or caches) 
    for an element node which are stored in the element node object during parsing.
    """

    def __init__ (self, filePath, absUrl, treeWrapper, elementWrapperClass):
        """Constructor for this class
        
        Input parameter:
            filePath:      contains the file path of the corresponding XML file
            absUrl:        contains the absolute URL of the corresponding XML file
        """
        self.filePath            = filePath
        self.absUrl              = absUrl
        self.baseUrlStack        = [absUrl, ]
        self.treeWrapper         = treeWrapper
        self.elementWrapperClass = elementWrapperClass


    def startElementHandler (self, curNode, startLineNumber, curNs, attributes=[]):
        """Called by the XML parser at creation of an element node.
        
        Input parameter:
            curNode:          current element node
            startLineNumber:  first line number of the element tag in XML file
            curNs:            namespaces visible for this element node
            attributes:       list of attributes and their values for this element node 
                              (same sequence as int he XML file)
        """
        
        elementWrapper              = self.elementWrapperClass(curNode, self.treeWrapper, curNs, initAttrSeq=0)
        
        elementWrapper.baseUrl = self.__getBaseUrl(elementWrapper)
        elementWrapper.absUrl  = self.absUrl
        elementWrapper.filePath = self.filePath
        elementWrapper.startLineNumber = startLineNumber
        elementWrapper.curNs.extend ([("xml", XML_NAMESPACE), ("xmlns", XMLNS_NAMESPACE)])

        if attributes != []:
            for i in range (0, len(attributes), 2):
                elementWrapper.attributeSequence.append(attributes[i])
        else:
            attrList = elementWrapper.getAttributeDict().keys()
            attrList.sort()
            elementWrapper.attributeSequence.extend (attrList)

        self.baseUrlStack.insert (0, elementWrapper.baseUrl)


    def endElementHandler (self, curNode, endLineNumber):
        """Called by the XML parser after creation of an element node.
        
        Input parameter:
            curNode:          current element node
            endLineNumber:    last line number of the element tag in XML file
        """
        curNode.xmlIfExtElementWrapper.endLineNumber = endLineNumber
        self.baseUrlStack.pop (0)


    def __getBaseUrl (self, elementWrapper):
        """Retrieve base URL for the given element node.
        
        Input parameter:
            elementWrapper:    wrapper of current element node
        """
        nsNameBaseAttr = NsNameTupleFactory ((XML_NAMESPACE, "base"))
        if elementWrapper.hasAttribute(nsNameBaseAttr):
            return convertToAbsUrl (elementWrapper.getAttribute(nsNameBaseAttr), self.baseUrlStack[0])
        else:
            return self.baseUrlStack[0]

