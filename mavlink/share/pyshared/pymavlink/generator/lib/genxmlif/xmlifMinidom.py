#
# genxmlif, Release 0.9.0
# file: xmlifMinidom.py
#
# XML interface class to Python standard minidom
#
# history:
# 2005-04-25 rl   created
# 2007-07-02 rl   complete re-design, internal wrapper 
#                 for DOM trees and elements introduced
# 2008-07-01 rl   Limited support of XInclude added
#
# Copyright (c) 2005-2008 by Roland Leuthe.  All rights reserved.
#
# --------------------------------------------------------------------
# The generix XML interface is
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

import string
import urllib
from xml.dom              import Node, XMLNS_NAMESPACE
from xml.dom.expatbuilder import ExpatBuilderNS
from xml.parsers.expat    import ExpatError
from ..genxmlif             import XMLIF_MINIDOM, GenXmlIfError
from xmlifUtils           import convertToAbsUrl, NsNameTupleFactory
from xmlifDom             import XmlInterfaceDom, InternalDomTreeWrapper, InternalDomElementWrapper, XmlIfBuilderExtensionDom


class XmlInterfaceMinidom (XmlInterfaceDom):
    """Derived interface class for handling of minidom parser.
    
    For description of the interface methods see xmlifbase.py.
    """

    def __init__ (self, verbose, useCaching, processXInclude):
        XmlInterfaceDom.__init__ (self, verbose, useCaching, processXInclude)
        self.xmlIfType = XMLIF_MINIDOM
        if self.verbose:
            print "Using minidom interface module..."


    def createXmlTree (self, namespace, xmlRootTagName, attributeDict={}, publicId=None, systemId=None):
        from xml.dom.minidom import getDOMImplementation
        domImpl = getDOMImplementation()
        doctype = domImpl.createDocumentType(xmlRootTagName, publicId, systemId)
        domTree = domImpl.createDocument(namespace, xmlRootTagName, doctype)
        treeWrapper = self.treeWrapperClass(self, InternalMinidomTreeWrapper(domTree), self.useCaching)

        intRootNodeWrapper = InternalMinidomElementWrapper(domTree.documentElement, treeWrapper.getTree())
        rootNodeWrapper = self.elementWrapperClass (intRootNodeWrapper, treeWrapper, []) # TODO: namespace handling
        for attrName, attrValue in attributeDict.items():
            rootNodeWrapper.setAttribute (attrName, attrValue)

        return treeWrapper


    def parse (self, file, baseUrl="", internalOwnerDoc=None):
        absUrl = convertToAbsUrl(file, baseUrl)
        fp     = urllib.urlopen (absUrl)
        try:
            builder = ExtExpatBuilderNS(file, absUrl, self)
            tree = builder.parseFile(fp)

            # XInclude support
            if self.processXInclude:
                if internalOwnerDoc == None: 
                    internalOwnerDoc = builder.treeWrapper.getTree()
                self.xInclude (builder.treeWrapper.getRootNode(), absUrl, internalOwnerDoc)

            fp.close()
        except ExpatError, errInst:
            fp.close()
            raise GenXmlIfError, "%s: ExpatError: %s" %(file, str(errInst))

        return builder.treeWrapper


    def parseString (self, text, baseUrl="", internalOwnerDoc=None):
        absUrl = convertToAbsUrl ("", baseUrl)
        try:
            builder = ExtExpatBuilderNS("", absUrl, self)
            builder.parseString (text)

            # XInclude support
            if self.processXInclude:
                if internalOwnerDoc == None: 
                    internalOwnerDoc = builder.treeWrapper.getTree()
                self.xInclude (builder.treeWrapper.getRootNode(), absUrl, internalOwnerDoc)
        except ExpatError, errInst:
            raise GenXmlIfError, "%s: ExpatError: %s" %(baseUrl, str(errInst))

        return builder.treeWrapper



class InternalMinidomTreeWrapper (InternalDomTreeWrapper):
    """Internal wrapper for a minidom Document class.
    """
    
    def __init__ (self, document):
        InternalDomTreeWrapper.__init__(self, document)
        self.internalElementWrapperClass = InternalMinidomElementWrapper



class InternalMinidomElementWrapper (InternalDomElementWrapper):
    """Internal Wrapper for a Dom Element class.
    """

    def xmlIfExtGetAttributeDict (self):
        """Return a dictionary with all attributes of this element."""
        attribDict = {}
        for attrNameNS, attrNodeOrValue in self.element.attributes.itemsNS():
            attribDict[NsNameTupleFactory(attrNameNS)] = attrNodeOrValue
                
        return attribDict



class ExtExpatBuilderNS (ExpatBuilderNS, XmlIfBuilderExtensionDom):
    """Extended Expat Builder class derived from ExpatBuilderNS.
    
    Extended to store related line numbers, file/URL names and 
    defined namespaces in the node object.
    """

    def __init__ (self, filePath, absUrl, xmlIf):
        ExpatBuilderNS.__init__(self)
        internalMinidomTreeWrapper = InternalMinidomTreeWrapper(self.document)
        self.treeWrapper = xmlIf.treeWrapperClass(self, internalMinidomTreeWrapper, xmlIf.useCaching)
        XmlIfBuilderExtensionDom.__init__(self, filePath, absUrl, self.treeWrapper, xmlIf.elementWrapperClass)

        # set EndNamespaceDeclHandler, currently not used by minidom
        self.getParser().EndNamespaceDeclHandler = self.end_namespace_decl_handler
        self.curNamespaces = []


    def start_element_handler(self, name, attributes):
        ExpatBuilderNS.start_element_handler(self, name, attributes)

        # use attribute format {namespace}localName
        attrList = []
        for i in range (0, len(attributes), 2):
            attrName = attributes[i]
            attrNameSplit = string.split(attrName, " ")
            if len(attrNameSplit) > 1:
                attrName = (attrNameSplit[0], attrNameSplit[1])
            attrList.extend([attrName, attributes[i+1]])
        
        internalMinidomElementWrapper = InternalMinidomElementWrapper(self.curNode, self.treeWrapper.getTree())
        XmlIfBuilderExtensionDom.startElementHandler (self, internalMinidomElementWrapper, self.getParser().ErrorLineNumber, self.curNamespaces[:], attrList)

        if self.curNode.parentNode.nodeType == Node.DOCUMENT_NODE:
            for namespace in self.curNamespaces:
                if namespace[0] != None:
                    internalMinidomElementWrapper.xmlIfExtElementWrapper.attributeSequence.append((XMLNS_NAMESPACE, namespace[0]))
                else:
                    internalMinidomElementWrapper.xmlIfExtElementWrapper.attributeSequence.append("xmlns")
#                internalMinidomElementWrapper.xmlIfExtElementWrapper.setAttribute((XMLNS_NAMESPACE, namespace[0]), namespace[1])


    def end_element_handler(self, name):
        XmlIfBuilderExtensionDom.endElementHandler (self, self.curNode.xmlIfExtInternalWrapper, self.getParser().ErrorLineNumber)
        ExpatBuilderNS.end_element_handler(self, name)


    def start_namespace_decl_handler(self, prefix, uri):
        ExpatBuilderNS.start_namespace_decl_handler(self, prefix, uri)
        self.curNamespaces.insert(0, (prefix, uri))


    def end_namespace_decl_handler(self, prefix):
        assert self.curNamespaces.pop(0)[0] == prefix, "implementation confused"

