#
# genxmlif, Release 0.9.0
# file: xmlifDom.py
#
# XML interface base class for Python DOM implementations
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

import string
import copy
import urllib
from types                import TupleType
from xml.dom              import Node, getDOMImplementation, XMLNS_NAMESPACE
from ..genxmlif             import XINC_NAMESPACE, GenXmlIfError
from xmlifUtils           import nsNameToQName, processWhitespaceAction, collapseString, NsNameTupleFactory, convertToAbsUrl
from xmlifBase            import XmlIfBuilderExtensionBase
from xmlifApi             import XmlInterfaceBase


class XmlInterfaceDom (XmlInterfaceBase):
    """Derived interface class for handling of DOM parsers.
    
    For description of the interface methods see xmlifbase.py.
    """
    
    def xInclude (self, elementWrapper, baseUrl, ownerDoc):
        filePath = elementWrapper.getFilePath()
        for childElementWrapper in elementWrapper.getChildren():
            line = childElementWrapper.getStartLineNumber()
            if childElementWrapper.getNsName() == (XINC_NAMESPACE, "include"):
                href = childElementWrapper["href"]
                parse = childElementWrapper.getAttributeOrDefault ("parse", "xml")
                encoding = childElementWrapper.getAttribute ("encoding")
                if self.verbose:
                    print "Xinclude: %s" %href
                try:
                    if parse == "xml":
                        subTreeWrapper = self.parse (href, baseUrl, ownerDoc)
                        elementWrapper.replaceChildBySubtree (childElementWrapper, subTreeWrapper)
                    elif parse == "text":
                        absUrl = convertToAbsUrl (href, baseUrl)
                        fp = urllib.urlopen (absUrl)
                        data = fp.read()
                        if encoding:
                            data = data.decode(encoding)
                        newTextNode = ownerDoc.xmlIfExtCreateTextNode(data)
                        elementWrapper.element.element.insertBefore (newTextNode, childElementWrapper.element.element)
                        elementWrapper.removeChild (childElementWrapper)
                        fp.close()
                    else:
                        raise GenXmlIfError, "%s: line %s: XIncludeError: Invalid 'parse' Attribut: '%s'" %(filePath, line, parse)
                except IOError, errInst:
                    raise GenXmlIfError, "%s: line %s: IOError: %s" %(filePath, line, str(errInst))
            elif childElementWrapper.getNsName() == (XINC_NAMESPACE, "fallback"):
                raise GenXmlIfError, "%s: line %s: XIncludeError: xi:fallback tag must be child of xi:include" %(filePath, line)
            else:
                self.xInclude(childElementWrapper, baseUrl, ownerDoc)



class InternalDomTreeWrapper:
    """Internal wrapper for a DOM Document class.
    """
    def __init__ (self, document):
        self.document   = document
    
    def xmlIfExtGetRootNode (self):
        domNode = self.document
        if domNode.nodeType == Node.DOCUMENT_NODE:
            return domNode.documentElement.xmlIfExtInternalWrapper
        elif domNode.nodeType == Node.DOCUMENT_FRAGMENT_NODE:
            for node in domNode.childNodes:
                if node.nodeType == Node.ELEMENT_NODE:
                    return node.xmlIfExtInternalWrapper
            else:
                return None
        else:
            return None


    def xmlIfExtCreateElement (self, nsName, attributeDict, curNs):
        elementNode = self.document.createElementNS (nsName[0], nsName[1])
        intElementWrapper = self.internalElementWrapperClass(elementNode, self)
        for attrName, attrValue in attributeDict.items():
            intElementWrapper.xmlIfExtSetAttribute (NsNameTupleFactory(attrName), attrValue, curNs)
        return intElementWrapper


    def xmlIfExtCreateTextNode (self, data):
        return self.document.createTextNode(data)
    

    def xmlIfExtImportNode (self, node):
        return self.document.importNode (node, 0) 
        

    def xmlIfExtCloneTree (self, rootElementCopy):
        domImpl = getDOMImplementation()
#        documentCopy = domImpl.createDocument(rootElementCopy.xmlIfExtGetNamespaceURI(), rootElementCopy.xmlIfExtGetTagName(), None)
        documentCopy = domImpl.createDocument(None, None, None)
#        documentCopy = copy.copy(self.document)
        documentCopy.documentElement = rootElementCopy.element
        return self.__class__(documentCopy)

    

#########################################################
# Internal Wrapper class for a Dom Element class

class InternalDomElementWrapper:
    """Internal Wrapper for a Dom Element class.
    """
    
    def __init__ (self, element, internalDomTreeWrapper):
        self.element = element
        element.xmlIfExtInternalWrapper = self
        self.internalDomTreeWrapper = internalDomTreeWrapper
        

    def xmlIfExtUnlink (self):
        self.xmlIfExtElementWrapper = None
        

    def xmlIfExtCloneNode (self):
        nodeCopy = self.__class__(self.element.cloneNode(deep=0), self.internalDomTreeWrapper)
        for childTextNode in self.__xmlIfExtGetChildTextNodes():
            childTextNodeCopy = childTextNode.cloneNode(0)
            nodeCopy.element.appendChild (childTextNodeCopy)
#        for nsAttrName, attrValue in self.xmlIfExtGetAttributeDict().items():
#            nodeCopy.xmlIfExtSetAttribute(nsAttrName, attrValue, self.xmlIfExtElementWrapper.getCurrentNamespaces())
        return nodeCopy

    
    def xmlIfExtGetTagName (self):
        return self.element.tagName


    def xmlIfExtGetNamespaceURI (self):
        return self.element.namespaceURI


    def xmlIfExtGetParentNode (self):
        parentNode = self.element.parentNode
        if parentNode.nodeType == Node.ELEMENT_NODE:
            return self.element.parentNode.xmlIfExtInternalWrapper
        else:
            return None

    
    def xmlIfExtSetParentNode (self, parentElement):
        pass # nothing to do since parent is provided by DOM interface
    

    def xmlIfExtGetChildren (self, tagFilter=None):
        # TODO: Handle also wildcard tagFilter = (namespace, None)
        children = filter (lambda e: (e.nodeType == Node.ELEMENT_NODE) and          # - only ELEMENTs
                                      (tagFilter == None or 
                                       (e.namespaceURI == tagFilter[0] and e.localName == tagFilter[1])), # - if tagFilter given --> check
                           self.element.childNodes )                                 # from element's nodes

        return map(lambda element: element.xmlIfExtInternalWrapper, children)


    def xmlIfExtGetFirstChild (self, tagFilter=None):
        children = self.xmlIfExtGetChildren (tagFilter)
        if children != []:
            return children[0]
        else:
            None


    def xmlIfExtGetElementsByTagName (self, tagFilter=('*','*')):
        elementList = self.element.getElementsByTagNameNS( tagFilter[0], tagFilter[1] )
        return map( lambda element: element.xmlIfExtInternalWrapper, elementList )


    def xmlIfExtGetIterator (self, tagFilter=('*','*')):
        elementList = []
        if tagFilter in (('*','*'), (self.element.namespaceURI, self.element.localName)):
            elementList.append(self.element)
        elementList.extend(self.element.getElementsByTagNameNS( tagFilter[0], tagFilter[1] ))
        return map( lambda element: element.xmlIfExtInternalWrapper, elementList )

    
    def xmlIfExtAppendChild (self, childElement):
        self.element.appendChild (childElement.element)


    def xmlIfExtInsertBefore (self, childElement, refChildElement):
        self.element.insertBefore (childElement.element, refChildElement.element)


    def xmlIfExtRemoveChild (self, childElement):
        self.element.removeChild (childElement.element)


    def xmlIfExtInsertSubtree (self, refChildElement, subTree, insertSubTreeRootNode):
        if insertSubTreeRootNode:
            childElementList = [subTree.xmlIfExtGetRootNode(),]
        else:
            childElementList = subTree.xmlIfExtGetRootNode().xmlIfExtGetChildren()

        for childElement in childElementList:
            if refChildElement != None:
                self.element.insertBefore(childElement.element, refChildElement.element)
            else:
                self.element.appendChild(childElement.element)


    def xmlIfExtGetAttributeDict (self):
        attribDict = {}
        for nsAttrName, attrNodeOrValue in self.element.attributes.items():
            attribDict[NsNameTupleFactory(nsAttrName)] = attrNodeOrValue.nodeValue
        return attribDict


    def xmlIfExtGetAttribute (self, nsAttrName):
        if self.element.attributes.has_key (nsAttrName):
            return self.element.getAttributeNS (nsAttrName[0], nsAttrName[1])
        elif nsAttrName[1] == "xmlns" and self.element.attributes.has_key(nsAttrName[1]):
            # workaround for minidom for correct access of xmlns attribute
            return self.element.getAttribute (nsAttrName[1])
        else:
            return None


    def xmlIfExtSetAttribute (self, nsAttrName, attributeValue, curNs):
        if nsAttrName[0] != None:
            qName = nsNameToQName (nsAttrName, curNs)
        else:
            qName = nsAttrName[1]
        
        self.element.setAttributeNS (nsAttrName[0], qName, attributeValue)


    def xmlIfExtRemoveAttribute (self, nsAttrName):
        self.element.removeAttributeNS (nsAttrName[0], nsAttrName[1])


    def xmlIfExtGetElementValueFragments (self, ignoreEmtpyStringFragments):
        elementValueList = []
        for childTextNode in self.__xmlIfExtGetChildTextNodes():
            elementValueList.append(childTextNode.data)
        if ignoreEmtpyStringFragments:
            elementValueList = filter (lambda s: collapseString(s) != "", elementValueList)
        if elementValueList == []:
            elementValueList = ["",]
        return elementValueList


    def xmlIfExtGetElementText (self):
        elementTextList = ["",]
        if self.element.childNodes != []:
            for childNode in self.element.childNodes:
                if childNode.nodeType in (Node.TEXT_NODE, Node.CDATA_SECTION_NODE):
                    elementTextList.append (childNode.data)
                else:
                    break
        return "".join(elementTextList)

    
    def xmlIfExtGetElementTailText (self):
        tailTextList = ["",]
        nextSib = self.element.nextSibling
        while nextSib:
            if nextSib.nodeType in (Node.TEXT_NODE, Node.CDATA_SECTION_NODE):
                tailTextList.append (nextSib.data)
                nextSib = nextSib.nextSibling
            else:
                break
        return "".join(tailTextList)
        

    def xmlIfExtSetElementValue (self, elementValue):
        if self.__xmlIfExtGetChildTextNodes() == []:
            textNode = self.internalDomTreeWrapper.xmlIfExtCreateTextNode (elementValue)
            self.element.appendChild (textNode)
        else:
            self.__xmlIfExtGetChildTextNodes()[0].data = elementValue
            if len (self.__xmlIfExtGetChildTextNodes()) > 1:
                for textNode in self.__xmlIfExtGetChildTextNodes()[1:]:
                    textNode.data = ""
            

    def xmlIfExtProcessWsElementValue (self, wsAction):
        textNodes = self.__xmlIfExtGetChildTextNodes()

        if len(textNodes) == 1:
            textNodes[0].data = processWhitespaceAction (textNodes[0].data, wsAction)
        elif len(textNodes) > 1:
            textNodes[0].data = processWhitespaceAction (textNodes[0].data, wsAction, rstrip=0)
            lstrip = 0
            if len(textNodes[0].data) > 0 and textNodes[0].data[-1] == " ":
                lstrip = 1
            for textNode in textNodes[1:-1]:
                textNode.data = processWhitespaceAction (textNode.data, wsAction, lstrip, rstrip=0)
                if len(textNode.data) > 0 and textNode.data[-1] == " ":
                    lstrip = 1
                else:
                    lstrip = 0
            textNodes[-1].data = processWhitespaceAction (textNodes[-1].data, wsAction, lstrip)


    ###############################################################
    # PRIVATE methods
    ###############################################################

    def __xmlIfExtGetChildTextNodes ( self ):
        """Return list of TEXT nodes."""
        return filter (lambda e: ( e.nodeType in (Node.TEXT_NODE, Node.CDATA_SECTION_NODE) ), # - only TEXT-NODES
                       self.element.childNodes)                         # from element's child nodes
        


class XmlIfBuilderExtensionDom (XmlIfBuilderExtensionBase):
    """XmlIf builder extension class for DOM parsers."""
    
    pass
