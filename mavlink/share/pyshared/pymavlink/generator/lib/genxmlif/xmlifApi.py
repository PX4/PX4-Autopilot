#
# genxmlif, Release 0.9.0
# file: xmlifapi.py
#
# API (interface) classes for generic interface package
#
# history:
# 2007-06-29 rl   created, classes extracted from xmlifbase.py
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
__date__    = "08. August 2008"
__version__ = "0.9.0"

import string
import os
import re
import copy
from types      import TupleType, StringTypes
from xml.dom    import EMPTY_PREFIX, EMPTY_NAMESPACE
from xmlifUtils import processWhitespaceAction, NsNameTupleFactory, splitQName, nsNameToQName, escapeCdata, escapeAttribute


########################################
# XML interface base class
# All not implemented methods have to be overloaded by the derived class!!
#

class XmlInterfaceBase:
    """XML interface base class.
    
    All not implemented methods have to be overloaded by the derived class!!
    """

    def __init__(self, verbose, useCaching, processXInclude):
        """Constructor of class XmlInterfaceBase.
        
        Input parameter:
            'verbose':         0 or 1: controls verbose print output for module genxmlif
            'useCaching':      0 or 1: controls usage of caching for module genxmlif
            'processXInclude': 0 or 1: controls XInclude processing during parsing
        """
        
        self.verbose         = verbose
        self.useCaching      = useCaching
        self.processXInclude = processXInclude

        # set default wrapper classes
        self.setTreeWrapperClass (XmlTreeWrapper)
        self.setElementWrapperClass (XmlElementWrapper)


    def createXmlTree (self, namespace, xmlRootTagName, attributeDict={}, publicId=None, systemId=None):
        """Create a new XML TreeWrapper object (wrapper for DOM document or elementtree).
        
        Input parameter:
            'namespace':      not yet handled (for future use)
            'xmlRootTagName': specifies the tag name of the root element
            'attributeDict':  contains the attributes of the root node (optional)
            'publicId':       forwarded to contained DOM tree (unused for elementtree)
            'systemId':       forwarded to contained DOM tree (unused for elementtree)
        Returns the created XML tree wrapper object.
        Method has to be implemented by derived classes!
        """
         
        raise NotImplementedError


    def parse (self, filePath, baseUrl="", ownerDoc=None):
        """Call the XML parser for 'file'.
        
        Input parameter:
            'filePath': a file path or an URI
            'baseUrl':  if specified, it is used e.g. as base path for schema files referenced inside the XML file.
            'ownerDoc': only used in case of 4DOM (forwarded to 4DOM parser). 
        Returns the respective XML tree wrapper object for the parsed XML file.
        Method has to be implemented by derived classes!
        """
        
        raise NotImplementedError


    def parseString (self, text, baseUrl="", ownerDoc=None):
        """Call the XML parser for 'text'.

        Input parameter:
            'text':     contains the XML string to be parsed
            'baseUrl':  if specified, it is used e.g. as base path for schema files referenced inside the XML string.
            'ownerDoc': only used in case of 4DOM (forwarded to 4DOM parser). 
        Returns the respective XML tree wrapper object for the parsed XML 'text' string.
        Method has to be implemented by derived classes!
        """
        raise NotImplementedError


    def setTreeWrapperClass (self, treeWrapperClass):
        """Set the tree wrapper class which shall be used by this interface.

        Input parameter:
            treeWrapperClass:     tree wrapper class
        """
        self.treeWrapperClass = treeWrapperClass


    def setElementWrapperClass (self, elementWrapperClass):
        """Set the element wrapper classes which shall be used by this interface.

        Input parameter:
            elementWrapperClass:  element wrapper class
        """
        self.elementWrapperClass = elementWrapperClass
        

    def getXmlIfType (self):
        """Retrieve the type of the XML interface."""
        return self.xmlIfType
    

########################################
# Tree wrapper API (interface class)
#

class XmlTreeWrapper:
    """XML tree wrapper API.

    Contains a DOM tree or an elementtree (depending on used XML parser)
    """

    def __init__(self, xmlIf, tree, useCaching):
        """Constructor of wrapper class XmlTreeWrapper.
        
        Input parameter:
            'xmlIf':      used XML interface class
            'tree':       DOM tree or elementtree which is wrapped by this object
            'useCaching': 1 if caching shall be used inside genxmlif, otherwise 0
        """
        self.xmlIf                   = xmlIf
        self.__tree                  = tree
        self.__useCaching            = useCaching


    def createElement (self, tupleOrLocalName, attributeDict=None, curNs=[]):
        """Create an ElementWrapper object.
        
        Input parameter:
            tupleOrLocalName: tag name of element node to be created 
                              (tuple of namespace and localName or only localName if no namespace is used)
            attributeDict:    attributes for this elements
            curNs:            namespaces for scope of this element
        Returns an ElementWrapper object containing the created element node.
        """
        nsName = NsNameTupleFactory(tupleOrLocalName)
        elementNode    = self.__tree.xmlIfExtCreateElement(nsName, attributeDict, curNs)
        return self.xmlIf.elementWrapperClass(elementNode, self, curNs)


    def cloneTree (self):
        """Creates a copy of a whole XML DOM tree."""
        rootElementWrapperCopy = self.getRootNode().cloneNode(deep=1)
        treeWrapperCopy = self.__class__(self.xmlIf, 
                                         self.__tree.xmlIfExtCloneTree(rootElementWrapperCopy.element), 
                                         self.__useCaching)
        for elementWrapper in rootElementWrapperCopy.getIterator():
            elementWrapper.treeWrapper = treeWrapperCopy
        return treeWrapperCopy
        
    
    def getRootNode (self):
        """Retrieve the wrapper object of the root element of the contained XML tree.
        
        Returns the ElementWrapper object of the root element.
        """
        return self.__tree.xmlIfExtGetRootNode().xmlIfExtElementWrapper


    def getTree (self):
        """Retrieve the contained XML tree.
        
        Returns the contained XML tree object (internal DOM tree wrapper or elementtree).
        """
        return self.__tree

    
    def printTree (self, prettyPrint=0, printElementValue=1, encoding=None):
        """Return the string representation of the contained XML tree.
        
        Input parameter:
            'prettyPrint':        aligns the columns of the attributes of childNodes
            'printElementValue':  controls if the lement values are printed or not.
        Returns a string with the string representation of the whole XML tree.
        """
        if not encoding:
            encoding = "utf-8"
        if encoding != "utf-8" and encoding != "us-ascii":
            text = "<?xml version='1.0' encoding='%s'?>\n" % encoding
        else:
            text = ""
        return text + self.getRootNode().printNode(deep=1, prettyPrint=prettyPrint, printElementValue=printElementValue, encoding=encoding)


    def useCaching (self):
        """Return 1 if caching should be used for the contained XML tree."""
        return self.__useCaching


    def setExternalCacheUsage (self, used):
        """Set external cache usage for the whole tree
           unlink commands are ignored if used by an external cache

           Input parameter:
               used:       0 or 1 (used by external cache)
        """
        self.getRootNode().setExternalCacheUsage (used, deep=1)

    
    def unlink (self):
        """Break circular references of the complete XML tree.
        
        To be called if the XML tree is not longer used => garbage collection!
        """
        self.getRootNode().unlink()
        

    def __str__ (self):
        """Return the string representation of the contained XML tree."""
        return self.printTree()



########################################
# Element wrapper API (interface class)
#

class XmlElementWrapper:
    """XML element wrapper API.

    Contains a XML element node
    All not implemented methods have to be overloaded by the derived class!!
    """

    def __init__(self, element, treeWrapper, curNs=[], initAttrSeq=1):
        """Constructor of wrapper class XmlElementWrapper.
        
        Input parameter:
            element:       XML element node which is wrapped by this object
            treeWrapper:   XML tree wrapper class the current element belongs to
            curNs:         namespaces for scope of this element
        """
        self.element                        = element
        self.element.xmlIfExtElementWrapper = self
        self.treeWrapper                    = treeWrapper
        self.nodeUsedByExternalCache        = 0
        
        if self.__useCaching():
            self.__childrenCache = {}
            self.__firstChildCache = {}
            self.__qNameAttrCache = {}

        self.baseUrl = None
        self.absUrl = None
        self.filePath = None
        self.startLineNumber = None
        self.endLineNumber = None
        self.curNs = curNs[:]
        self.attributeSequence = []

        if initAttrSeq:
            self.attributeSequence = self.getAttributeDict().keys()


    def unlink (self):
        """Break circular references of this element and its children."""
        for childWrapper in self.getChildren():
            childWrapper.unlink()
        if not self.isUsedByExternalCache():
            self.element.xmlIfExtUnlink()

        
    def cloneNode (self, deep, cloneCallback=None):
        """Create a copy of the current element wrapper.
           The reference to the parent node is set to None!"""
        elementCopy = self.element.xmlIfExtCloneNode()
        elementWrapperCopy = self.__class__(elementCopy, self.treeWrapper, initAttrSeq=0)
        elementWrapperCopy.treeWrapper = None
        elementWrapperCopy.baseUrl = self.baseUrl
        elementWrapperCopy.absUrl = self.absUrl
        elementWrapperCopy.filePath = self.filePath
        elementWrapperCopy.startLineNumber = self.startLineNumber
        elementWrapperCopy.endLineNumber = self.endLineNumber
        elementWrapperCopy.curNs = self.curNs[:]
        elementWrapperCopy.attributeSequence = self.attributeSequence[:]
        if cloneCallback: cloneCallback(elementWrapperCopy)
        if deep:
            for childElement in self.element.xmlIfExtGetChildren():
                childWrapperElementCopy = childElement.xmlIfExtElementWrapper.cloneNode(deep, cloneCallback)
                childWrapperElementCopy.element.xmlIfExtSetParentNode(elementWrapperCopy.element)
                elementWrapperCopy.element.xmlIfExtAppendChild(childWrapperElementCopy.element)
        return elementWrapperCopy
    
    
    def clearNodeCache (self):
        """Clear all caches used by this element wrapper which contains element wrapper references."""
        self.__clearChildrenCache()


    def isUsedByExternalCache (self):
        """Check if this node is used by an external cache.
           unlink commands are ignored if used by an external cache"""
        return self.nodeUsedByExternalCache
    
    
    def setExternalCacheUsage (self, used, deep=1):
        """Set external cache usage for this node and its children
           unlink commands are ignored if used by an external cache

           Input parameter:
               used:       0 or 1 (used by external cache)
               deep:       0 or 1: controls if the child elements are also marked as used by external cache
        """
        self.nodeUsedByExternalCache = used
        if deep:
            for childWrapper in self.getChildren():
                childWrapper.setExternalCacheUsage (used, deep)
        


    ##########################################################
    #  attributes of the current node can be accessed via key operator
    
    def __getitem__(self, tupleOrAttrName):
        """Attributes of the contained element node can be accessed via key operator.
        
        Input parameter:
            tupleOrAttrName: name of the attribute (tuple of namespace and attributeName or only attributeName)
        Returns the attribute value.
        """
        attrValue = self.getAttribute (tupleOrAttrName)
        if attrValue != None:
            return attrValue
        else:
            raise AttributeError, "Attribute %s not found!" %(repr(tupleOrAttrName))


    def __setitem__(self, tupleOrAttrName, attributeValue):
        """Attributes of the contained element node can be accessed via key operator.
        
        Input parameter:
            tupleOrAttrName: name of the attribute (tuple of namespace and attributeName or only attributeName)
            attributeValue:  attribute value to be set
        """
        self.setAttribute (tupleOrAttrName, attributeValue)


#++++++++++++ methods concerning the tag name ++++++++++++++++++++++++

    def getTagName (self):
        """Retrieve the (complete) tag name of the contained element node
        
        Returns the (complete) tag name of the contained element node
        """
        return self.element.xmlIfExtGetTagName()


    def getLocalName (self):
        """Retrieve the local name (without namespace) of the contained element node
        
        Returns the local name (without namespace) of the contained element node
        """
        
        try:
            return self.__localNameCache
        except:
            prefix, localName = splitQName (self.getTagName())
            if self.__useCaching():
                self.__localNameCache = localName
        return localName


    def getNamespaceURI (self):
        """Retrieve the namespace URI of the contained element node
        
        Returns the namespace URI of the contained element node (None if no namespace is used).
        """
        try:
            return self.__nsUriCache
        except:
            prefix = self.element.xmlIfExtGetNamespaceURI()
            if self.__useCaching():
                self.__nsUriCache = prefix 
            return prefix


    def getNsName (self):
        """Retrieve a tuple (namespace, localName) of the contained element node
        
        Returns a tuple (namespace, localName) of the contained element node (namespace is None if no namespace is used).
        """
        try:
            return self.__nsNameCache
        except:
            nsName = NsNameTupleFactory( (self.getNamespaceURI(), self.getLocalName()) )
            if self.__useCaching():
                self.__nsNameCache = nsName
            return nsName


    def getQName (self):
        """Retrieve a string prefix and localName of the contained element node

        Returns a string "prefix:localName" of the contained element node
        """
        return self.nsName2QName(self.getNsName())


    def getPrefix (self):
        """Retrieve the namespace prefix of the contained element node
        
        Returns the namespace prefix of the contained element node (None if no namespace is used).
        """
        return self.getNsPrefix(self.getNsName())


#++++++++++++ methods concerning print support ++++++++++++++++++++++++

    def __str__ (self):
        """Retrieve the textual representation of the contained element node."""
        return self.printNode()


    def printNode (self, indent="", deep=0, prettyPrint=0, attrMaxLengthDict={}, printElementValue=1, encoding=None):
        """Retrieve the textual representation of the contained element node.
        
        Input parameter:
            indent:             indentation to be used for string representation
            deep:               0 or 1: controls if the child element nodes are also printed
            prettyPrint:        aligns the columns of the attributes of childNodes
            attrMaxLengthDict:  dictionary containing the length of the attribute values (used for prettyprint)
            printElementValue:  0 or 1: controls if the element value is printed
        Returns the string representation
        """
        patternXmlTagShort = '''\
%(indent)s<%(qName)s%(attributeString)s/>%(tailText)s%(lf)s'''

        patternXmlTagLong = '''\
%(indent)s<%(qName)s%(attributeString)s>%(elementValueString)s\
%(lf)s%(subTreeString)s\
%(indent)s</%(qName)s>%(tailText)s%(lf)s'''
        
        subTreeStringList = []
        tailText = ""
        addIndent = ""
        lf = ""
        if deep:
            childAttrMaxLengthDict = {}
            if prettyPrint:
                for childNode in self.getChildren():
                    childNode.__updateAttrMaxLengthDict(childAttrMaxLengthDict)
                lf = "\n"
                addIndent = "    "
            for childNode in self.getChildren():
                subTreeStringList.append (childNode.printNode(indent + addIndent, deep, prettyPrint, childAttrMaxLengthDict, printElementValue))
            tailText = escapeCdata(self.element.xmlIfExtGetElementTailText(), encoding)
        
        attributeStringList = []
        for attrName in self.getAttributeList():
            attrValue = escapeAttribute(self.getAttribute(attrName), encoding)
            if prettyPrint:
                try:
                    align = attrMaxLengthDict[attrName]
                except:
                    align = len(attrValue)
            else:
                align = len(attrValue)
            qName = self.nsName2QName(attrName)
            attributeStringList.append (' %s="%s"%*s' %(qName, attrValue, align - len(attrValue), ""))
        attributeString = string.join (attributeStringList, "")

        qName = self.getQName()
        if printElementValue:
            if deep:
                elementValueString = escapeCdata(self.element.xmlIfExtGetElementText(), encoding)
            else:
                elementValueString = escapeCdata(self.getElementValue(ignoreEmtpyStringFragments=1), encoding)
        else:
            elementValueString = ""

        if subTreeStringList == [] and elementValueString == "":
            printPattern = patternXmlTagShort
        else:
            if subTreeStringList != []:
                subTreeString = string.join (subTreeStringList, "")
            else:
                subTreeString = ""
            printPattern = patternXmlTagLong
        return printPattern % vars()


#++++++++++++ methods concerning the parent of the current node ++++++++++++++++++++++++

    def getParentNode (self):
        """Retrieve the ElementWrapper object of the parent element node.

        Returns the ElementWrapper object of the parent element node.
        """
        parent = self.element.xmlIfExtGetParentNode()
        if parent != None:
            return parent.xmlIfExtElementWrapper
        else:
            return None


#++++++++++++ methods concerning the children of the current node ++++++++++++++++++++++++


    def getChildren (self, tagFilter=None):
        """Retrieve the ElementWrapper objects of the children element nodes.
        
        Input parameter:
            tagFilter: retrieve only the children with this tag name ('*' or None returns all children)
        Returns all children of this element node which match 'tagFilter' (list)
        """
        if tagFilter in (None, '*', (None, '*')):
            children = self.element.xmlIfExtGetChildren()
        elif tagFilter[1] == '*':
            # handle (namespace, '*')
            children = filter(lambda child:child.xmlIfExtElementWrapper.getNamespaceURI() == tagFilter[0], 
                              self.element.xmlIfExtGetChildren())
        else:
            nsNameFilter = NsNameTupleFactory(tagFilter)
            try:
                children = self.__childrenCache[nsNameFilter]
            except:
                children = self.element.xmlIfExtGetChildren(nsNameFilter)
                if self.__useCaching():
                    self.__childrenCache[nsNameFilter] = children

        return map(lambda child: child.xmlIfExtElementWrapper, children)


    def getChildrenNS (self, namespaceURI, tagFilter=None):
        """Retrieve the ElementWrapper objects of the children element nodes using a namespace.
        
        Input parameter:
            namespaceURI: the namespace URI of the children or None
            tagFilter:    retrieve only the children with this localName ('*' or None returns all children)
        Returns all children of this element node which match 'namespaceURI' and 'tagFilter' (list)
        """
        return self.getChildren((namespaceURI, tagFilter))


    def getChildrenWithKey (self, tagFilter=None, keyAttr=None, keyValue=None):
        """Retrieve the ElementWrapper objects of the children element nodes.
        
        Input parameter:
            tagFilter: retrieve only the children with this tag name ('*' or None returns all children)
            keyAttr:   name of the key attribute
            keyValue:  value of the key
        Returns all children of this element node which match 'tagFilter' (list)
        """
        children = self.getChildren(tagFilter)
        return filter(lambda child:child[keyAttr]==keyValue, children)
    
    
    def getFirstChild (self, tagFilter=None):
        """Retrieve the ElementWrapper objects of the first child element node.
        
        Input parameter:
            tagFilter: retrieve only the first child with this tag name ('*' or None: no filter)
        Returns the first child of this element node which match 'tagFilter'
        or None if no suitable child element was found
        """
        if tagFilter in (None, '*', (None, '*')):
            element = self.element.xmlIfExtGetFirstChild()
        elif tagFilter[1] == '*':
            # handle (namespace, '*')
            children = filter(lambda child:child.xmlIfExtElementWrapper.getNamespaceURI() == tagFilter[0], 
                              self.element.xmlIfExtGetChildren())
            try:
                element = children[0]
            except:
                element = None
        else:
            nsNameFilter = NsNameTupleFactory(tagFilter)
            try:
                element = self.__firstChildCache[nsNameFilter]
            except:
                element = self.element.xmlIfExtGetFirstChild(nsNameFilter)
                if self.__useCaching():
                    self.__firstChildCache[nsNameFilter] = element

        if element != None:
            return element.xmlIfExtElementWrapper
        else:
            return None


    def getFirstChildNS (self, namespaceURI, tagFilter=None):
        """Retrieve the ElementWrapper objects of the first child element node using a namespace.
        
        Input parameter:
            namespaceURI: the namespace URI of the children or None
            tagFilter:    retrieve only the first child with this localName ('*' or None: no filter)
        Returns the first child of this element node which match 'namespaceURI' and 'tagFilter'
        or None if no suitable child element was found
        """
        return self.getFirstChild ((namespaceURI, tagFilter))


    def getFirstChildWithKey (self, tagFilter=None, keyAttr=None, keyValue=None):
        """Retrieve the ElementWrapper objects of the children element nodes.
        
        Input parameter:
            tagFilter: retrieve only the children with this tag name ('*' or None returns all children)
            keyAttr:   name of the key attribute
            keyValue:  value of the key
        Returns all children of this element node which match 'tagFilter' (list)
        """
        children = self.getChildren(tagFilter)
        childrenWithKey = filter(lambda child:child[keyAttr]==keyValue, children)
        if childrenWithKey != []:
            return childrenWithKey[0]
        else:
            return None

    
    def getElementsByTagName (self, tagFilter=None):
        """Retrieve all descendant ElementWrapper object of current node whose tag name match 'tagFilter'.

        Input parameter:
            tagFilter: retrieve only the children with this tag name ('*' or None returns all descendants)
        Returns all descendants of this element node which match 'tagFilter' (list)
        """
        if tagFilter in (None, '*', (None, '*'), (None, None)):
            descendants = self.element.xmlIfExtGetElementsByTagName()
            
        elif tagFilter[1] == '*':
            # handle (namespace, '*')
            descendants = filter(lambda desc:desc.xmlIfExtElementWrapper.getNamespaceURI() == tagFilter[0], 
                                 self.element.xmlIfExtGetElementsByTagName())
        else:
            nsNameFilter = NsNameTupleFactory(tagFilter)
            descendants = self.element.xmlIfExtGetElementsByTagName(nsNameFilter)

        return map(lambda descendant: descendant.xmlIfExtElementWrapper, descendants)


    def getElementsByTagNameNS (self, namespaceURI, tagFilter=None):
        """Retrieve all descendant ElementWrapper object of current node whose tag name match 'namespaceURI' and 'tagFilter'.
        
        Input parameter:
            namespaceURI: the namespace URI of the descendants or None
            tagFilter:    retrieve only the descendants with this localName ('*' or None returns all descendants)
        Returns all descendants of this element node which match 'namespaceURI' and 'tagFilter' (list)
        """
        return self.getElementsByTagName((namespaceURI, tagFilter))


    def getIterator (self, tagFilter=None):
        """Creates a tree iterator.  The iterator loops over this element
           and all subelements, in document order, and returns all elements
           whose tag name match 'tagFilter'.

        Input parameter:
            tagFilter: retrieve only the children with this tag name ('*' or None returns all descendants)
        Returns all element nodes which match 'tagFilter' (list)
        """
        if tagFilter in (None, '*', (None, '*'), (None, None)):
            matchingElements = self.element.xmlIfExtGetIterator()
        elif tagFilter[1] == '*':
            # handle (namespace, '*')
            matchingElements = filter(lambda desc:desc.xmlIfExtElementWrapper.getNamespaceURI() == tagFilter[0], 
                                      self.element.xmlIfExtGetIterator())
        else:
            nsNameFilter = NsNameTupleFactory(tagFilter)
            matchingElements = self.element.xmlIfExtGetIterator(nsNameFilter)

        return map(lambda e: e.xmlIfExtElementWrapper, matchingElements)


    def appendChild (self, tupleOrLocalNameOrElement, attributeDict={}):
        """Append an element node to the children of the current node.

        Input parameter:
            tupleOrLocalNameOrElement: (namespace, localName) or tagName or ElementWrapper object of the new child
            attributeDict:             attribute dictionary containing the attributes of the new child (optional)
        If not an ElementWrapper object is given, a new ElementWrapper object is created with tupleOrLocalName
        Returns the ElementWrapper object of the new child.
        """
        if not isinstance(tupleOrLocalNameOrElement, self.__class__):
            childElementWrapper = self.__createElement (tupleOrLocalNameOrElement, attributeDict)
        else:
            childElementWrapper = tupleOrLocalNameOrElement
        self.element.xmlIfExtAppendChild (childElementWrapper.element)
        self.__clearChildrenCache(childElementWrapper.getNsName())
        return childElementWrapper


    def insertBefore (self, tupleOrLocalNameOrElement, refChild, attributeDict={}):
        """Insert an child element node before the given reference child of the current node.

        Input parameter:
            tupleOrLocalNameOrElement: (namespace, localName) or tagName or ElementWrapper object of the new child
            refChild:                  reference child ElementWrapper object
            attributeDict:             attribute dictionary containing the attributes of the new child (optional)
        If not an ElementWrapper object is given, a new ElementWrapper object is created with tupleOrLocalName
        Returns the ElementWrapper object of the new child.
        """
        if not isinstance(tupleOrLocalNameOrElement, self.__class__):
            childElementWrapper = self.__createElement (tupleOrLocalNameOrElement, attributeDict)
        else:
            childElementWrapper = tupleOrLocalNameOrElement
        if refChild == None:
            self.appendChild (childElementWrapper)
        else:
            self.element.xmlIfExtInsertBefore(childElementWrapper.element, refChild.element)
            self.__clearChildrenCache(childElementWrapper.getNsName())
        return childElementWrapper


    def removeChild (self, childElementWrapper):
        """Remove the given child element node from the children of the current node.

        Input parameter:
            childElementWrapper:  ElementWrapper object to be removed
        """
        self.element.xmlIfExtRemoveChild(childElementWrapper.element)
        self.__clearChildrenCache(childElementWrapper.getNsName())


    def insertSubtree (self, refChildWrapper, subTreeWrapper, insertSubTreeRootNode=1):
        """Insert the given subtree before 'refChildWrapper' ('refChildWrapper' is not removed!)
        
        Input parameter:
            refChildWrapper:       reference child ElementWrapper object
            subTreeWrapper:        subtree wrapper object which contains the subtree to be inserted
            insertSubTreeRootNode: if 1, root node of subtree is inserted into parent tree, otherwise not
        """ 
        if refChildWrapper != None:
            self.element.xmlIfExtInsertSubtree (refChildWrapper.element, subTreeWrapper.getTree(), insertSubTreeRootNode)
        else:
            self.element.xmlIfExtInsertSubtree (None, subTreeWrapper.getTree(), insertSubTreeRootNode)
        self.__clearChildrenCache()



    def replaceChildBySubtree (self, childElementWrapper, subTreeWrapper, insertSubTreeRootNode=1):
        """Replace child element node by XML subtree (e.g. expanding included XML files)

        Input parameter:
            childElementWrapper:   ElementWrapper object to be replaced
            subTreeWrapper:        XML subtree wrapper object to  be inserted
            insertSubTreeRootNode: if 1, root node of subtree is inserted into parent tree, otherwise not
        """
        self.insertSubtree (childElementWrapper, subTreeWrapper, insertSubTreeRootNode)
        self.removeChild(childElementWrapper)


#++++++++++++ methods concerning the attributes of the current node ++++++++++++++++++++++++

    def getAttributeDict (self):
        """Retrieve a dictionary containing all attributes of the current element node.
        
        Returns a dictionary (copy) containing all attributes of the current element node.
        """
        return self.element.xmlIfExtGetAttributeDict()


    def getAttributeList (self):
        """Retrieve a list containing all attributes of the current element node
           in the sequence specified in the input XML file.
        
        Returns a list (copy) containing all attributes of the current element node
        in the sequence specified in the input XML file (TODO: does currently not work for 4DOM/pyXML interface).
        """
        attrList = map(lambda a: NsNameTupleFactory(a), self.attributeSequence)
        return attrList


    def getAttribute (self, tupleOrAttrName):
        """Retrieve an attribute value of the current element node.
        
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
        Returns the value of the specified attribute.
        """
        nsName = NsNameTupleFactory(tupleOrAttrName)
        return self.element.xmlIfExtGetAttribute(nsName)


    def getAttributeOrDefault (self, tupleOrAttrName, defaultValue):
        """Retrieve an attribute value of the current element node or the given default value if the attribute doesn't exist.
        
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
        Returns the value of the specified attribute or the given default value if the attribute doesn't exist.
        """
        attributeValue = self.getAttribute (tupleOrAttrName)
        if attributeValue == None:
            attributeValue = defaultValue
        return attributeValue


    def getQNameAttribute (self, tupleOrAttrName):
        """Retrieve a QName attribute value of the current element node.
        
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
        Returns the value of the specified QName attribute as tuple (namespace, localName),
        i.e. the prefix is converted into the corresponding namespace value.
        """
        nsNameAttrName = NsNameTupleFactory(tupleOrAttrName)
        try:
            return self.__qNameAttrCache[nsNameAttrName]
        except:
            qNameValue = self.getAttribute (nsNameAttrName)
            nsNameValue = self.qName2NsName(qNameValue, useDefaultNs=1)
            if self.__useCaching():
                self.__qNameAttrCache[nsNameAttrName] = nsNameValue
            return nsNameValue


    def hasAttribute (self, tupleOrAttrName):
        """Checks if the requested attribute exist for the current element node.
        
        Returns 1 if the attribute exists, otherwise 0.
        """
        nsName = NsNameTupleFactory(tupleOrAttrName)
        attrValue = self.element.xmlIfExtGetAttribute(nsName)
        if attrValue != None:
            return 1
        else:
            return 0


    def setAttribute (self, tupleOrAttrName, attributeValue):
        """Sets an attribute value of the current element node. 
        If the attribute does not yet exist, it will be created.
               
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
            attributeValue:   attribute value to be set
        """
        if not isinstance(attributeValue, StringTypes):
            raise TypeError, "%s (attribute %s) must be a string!" %(repr(attributeValue), repr(tupleOrAttrName))

        nsNameAttrName = NsNameTupleFactory(tupleOrAttrName)
        if nsNameAttrName not in self.attributeSequence:
            self.attributeSequence.append(nsNameAttrName)

        if self.__useCaching():
            if self.__qNameAttrCache.has_key(nsNameAttrName):
                del self.__qNameAttrCache[nsNameAttrName]

        self.element.xmlIfExtSetAttribute(nsNameAttrName, attributeValue, self.getCurrentNamespaces())


    def setAttributeDefault (self, tupleOrAttrName, defaultValue):
        """Create attribute and set value to default if it does not yet exist for the current element node. 
        If the attribute is already existing nothing is done.
               
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
            defaultValue:     default attribute value to be set
        """
        if not self.hasAttribute(tupleOrAttrName):
            self.setAttribute(tupleOrAttrName, defaultValue)


    def removeAttribute (self, tupleOrAttrName):
        """Removes an attribute from the current element node. 
        No exception is raised if there is no matching attribute.
               
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
        """
        nsNameAttrName = NsNameTupleFactory(tupleOrAttrName)

        if self.__useCaching():
            if self.__qNameAttrCache.has_key(nsNameAttrName):
                del self.__qNameAttrCache[nsNameAttrName]

        self.element.xmlIfExtRemoveAttribute(nsNameAttrName)


    def processWsAttribute (self, tupleOrAttrName, wsAction):
        """Process white space action for the specified attribute according to requested 'wsAction'.
        
        Input parameter:
            tupleOrAttrName:  tuple '(namespace, attributeName)' or 'attributeName' if no namespace is used
            wsAction:         'collapse':  substitute multiple whitespace characters by a single ' '
                              'replace':   substitute each whitespace characters by a single ' '
        """
        attributeValue = self.getAttribute(tupleOrAttrName)
        newValue = processWhitespaceAction (attributeValue, wsAction)
        if newValue != attributeValue:
            self.setAttribute(tupleOrAttrName, newValue)
        return newValue
    

#++++++++++++ methods concerning the content of the current node ++++++++++++++++++++++++

    def getElementValue (self, ignoreEmtpyStringFragments=0):
        """Retrieve the content of the current element node.
        
        Returns the content of the current element node as string.
        The content of multiple text nodes / CDATA nodes are concatenated to one string.

        Input parameter:
            ignoreEmtpyStringFragments:   if 1, text nodes containing only whitespaces are ignored
        """
        return "".join (self.getElementValueFragments(ignoreEmtpyStringFragments))


    def getElementValueFragments (self, ignoreEmtpyStringFragments=0):
        """Retrieve the content of the current element node as value fragment list.
        
        Returns the content of the current element node as list of string fragments.
        Each list element represents one text nodes / CDATA node.

        Input parameter:
            ignoreEmtpyStringFragments:   if 1, text nodes containing only whitespaces are ignored
        
        Method has to be implemented by derived classes!
        """
        return self.element.xmlIfExtGetElementValueFragments (ignoreEmtpyStringFragments)


    def setElementValue (self, elementValue):
        """Set the content of the current element node.
        
        Input parameter:
            elementValue:   string containing the new element value
        If multiple text nodes / CDATA nodes are existing, 'elementValue' is set 
        for the first text node / CDATA node. All other text nodes /CDATA nodes are set to ''. 
        """
        self.element.xmlIfExtSetElementValue(elementValue)


    def processWsElementValue (self, wsAction):
        """Process white space action for the content of the current element node according to requested 'wsAction'.
        
        Input parameter:
            wsAction:         'collapse':  substitute multiple whitespace characters by a single ' '
                              'replace':   substitute each whitespace characters by a single ' '
        """
        self.element.xmlIfExtProcessWsElementValue(wsAction)
        return self.getElementValue()
        

#++++++++++++ methods concerning the info about the current node in the XML file ++++++++++++++++++++


    def getStartLineNumber (self):
        """Retrieve the start line number of the current element node.
        
        Returns the start line number of the current element node in the XML file
        """
        return self.startLineNumber


    def getEndLineNumber (self):
        """Retrieve the end line number of the current element node.
        
        Returns the end line number of the current element node in the XML file
        """
        return self.endLineNumber


    def getAbsUrl (self):
        """Retrieve the absolute URL of the XML file the current element node belongs to.
        
        Returns the absolute URL of the XML file the current element node belongs to.
        """
        return self.absUrl


    def getBaseUrl (self):
        """Retrieve the base URL of the XML file the current element node belongs to.
        
        Returns the base URL of the XML file the current element node belongs to.
        """
        return self.baseUrl


    def getFilePath (self):
        """Retrieve the file path of the XML file the current element node belongs to.
        
        Returns the file path of the XML file the current element node belongs to.
        """
        return self.filePath


    def getLocation (self, end=0, fullpath=0):
        """Retrieve a string containing file name and line number of the current element node.
        
        Input parameter:
            end:      1 if end line number shall be shown, 0 for start line number
            fullpath: 1 if the full path of the XML file shall be shown, 0 for only the file name
        Returns a string containing file name and line number of the current element node.
        (e.g. to be used for traces or error messages)
        """
        lineMethod = (self.getStartLineNumber, self.getEndLineNumber)
        pathFunc = (os.path.basename, os.path.abspath)
        return "%s, %d" % (pathFunc[fullpath](self.getFilePath()), lineMethod[end]())


#++++++++++++ miscellaneous methods concerning namespaces ++++++++++++++++++++


    def getCurrentNamespaces (self):
        """Retrieve the namespace prefixes visible for the current element node
        
        Returns a list of the namespace prefixes visible for the current node.
        """
        return self.curNs


    def qName2NsName (self, qName, useDefaultNs):
        """Convert a qName 'prefix:localName' to a tuple '(namespace, localName)'.
        
        Input parameter:
            qName:         qName to be converted
            useDefaultNs:  1 if default namespace shall be used
        Returns the corresponding tuple '(namespace, localName)' for 'qName'.
        """
        if qName != None:
            qNamePrefix, qNameLocalName = splitQName (qName)
            for prefix, namespaceURI in self.getCurrentNamespaces():
                if qNamePrefix == prefix:
                    if prefix != EMPTY_PREFIX or useDefaultNs:
                        nsName = (namespaceURI, qNameLocalName)
                        break
            else:
                if qNamePrefix == None:
                    nsName = (EMPTY_NAMESPACE, qNameLocalName)
                else:
                    raise ValueError, "Namespace prefix '%s' not bound to a namespace!" % (qNamePrefix)
        else:
            nsName = (None, None)
        return NsNameTupleFactory(nsName)


    def nsName2QName (self, nsLocalName):
        """Convert a tuple '(namespace, localName)' to a string 'prefix:localName'
        
        Input parameter:
            nsLocalName:   tuple '(namespace, localName)' to be converted
        Returns the corresponding string 'prefix:localName' for 'nsLocalName'.
        """
        qName = nsNameToQName (nsLocalName, self.getCurrentNamespaces())
        if qName == "xmlns:None": qName = "xmlns"
        return qName


    def getNamespace (self, qName):
        """Retrieve namespace for a qName 'prefix:localName'.
        
        Input parameter:
            qName:         qName 'prefix:localName'
        Returns the corresponding namespace for the prefix of 'qName'.
        """
        if qName != None:
            qNamePrefix, qNameLocalName = splitQName (qName)
            for prefix, namespaceURI in self.getCurrentNamespaces():
                if qNamePrefix == prefix:
                    namespace = namespaceURI
                    break
            else:
                if qNamePrefix == None:
                    namespace = EMPTY_NAMESPACE
                else:
                    raise LookupError, "Namespace for QName '%s' not found!" % (qName)
        else:
            namespace = EMPTY_NAMESPACE
        return namespace


    def getNsPrefix (self, nsLocalName):
        """Retrieve prefix for a tuple '(namespace, localName)'.
        
        Input parameter:
            nsLocalName:     tuple '(namespace, localName)'
        Returns the corresponding prefix for the namespace of 'nsLocalName'.
        """
        ns = nsLocalName[0]
        for prefix, namespace in self.getCurrentNamespaces():
            if ns == namespace:
                return prefix
        else:
            if ns == None:
                return None
            else:
                raise LookupError, "Prefix for namespaceURI '%s' not found!" % (ns)


#++++++++++++ limited XPath support ++++++++++++++++++++

    def getXPath (self, xPath, namespaceRef=None, useDefaultNs=1, attrIgnoreList=[]):
        """Retrieve node list or attribute list for specified XPath
        
        Input parameter:
            xPath:           string containing xPath specification
            namespaceRef:    scope for namespaces (default is own element node)
            useDefaultNs:    1, if default namespace shall be used if no prefix is available
            attrIgnoreList:  list of attributes to be ignored if wildcard is specified for attributes

        Returns all nodes which match xPath specification or
        list of attribute values if xPath specifies an attribute
        """
        return self.getXPathList(xPath, namespaceRef, useDefaultNs, attrIgnoreList)[0]


    def getXPathList (self, xPath, namespaceRef=None, useDefaultNs=1, attrIgnoreList=[]):
        """Retrieve node list or attribute list for specified XPath
        
        Input parameter:
            xPath:           string containing xPath specification
            namespaceRef:    scope for namespaces (default is own element node)
            useDefaultNs:    1, if default namespace shall be used if no prefix is available
            attrIgnoreList:  list of attributes to be ignored if wildcard is specified for attributes

        Returns tuple (completeChildList, attrNodeList, attrNsNameFirst).
        completeChildList: contains all child node which match xPath specification or
                           list of attribute values if xPath specifies an attribute
        attrNodeList:      contains all child nodes where the specified attribute was found
        attrNsNameFirst:   contains the name of the first attribute which was found
        TODO: Re-design namespace and attribute handling of this method
        """
        reChild     = re.compile('child *::')
        reAttribute = re.compile('attribute *::')
        if namespaceRef == None: namespaceRef = self
        xPath = reChild.sub('./', xPath)
        xPath = reAttribute.sub('@', xPath)
        xPathList = string.split (xPath, "|")
        completeChildDict = {}
        completeChildList = []
        attrNodeList = []
        attrNsNameFirst = None
        for xRelPath in xPathList:
            xRelPath = string.strip(xRelPath)
            descendantOrSelf = 0
            if xRelPath[:3] == ".//":
                descendantOrSelf = 1
                xRelPath = xRelPath[3:]
            xPathLocalStepList = string.split (xRelPath, "/")
            childList = [self, ]
            for localStep in xPathLocalStepList:
                localStep = string.strip(localStep)
                stepChildList = []
                if localStep == "":
                    raise IOError ("Invalid xPath '%s'!" %(xRelPath))
                elif localStep == ".":
                    continue
                elif localStep[0] == '@':
                    if len(localStep) == 1:
                        raise ValueError ("Attribute name is missing in xPath!")
                    if descendantOrSelf:
                        childList = self.getElementsByTagName()
                    attrName = localStep[1:]
                    for childNode in childList:
                        if attrName == '*':
                            attrNodeList.append (childNode)
                            attrDict = childNode.getAttributeDict()
                            for attrIgnore in attrIgnoreList:
                                if attrDict.has_key(attrIgnore):
                                    del attrDict[attrIgnore]
                            stepChildList.extend(attrDict.values())
                            try:
                                attrNsNameFirst = attrDict.keys()[0]
                            except:
                                pass
                        else:
                            attrNsName = namespaceRef.qName2NsName (attrName, useDefaultNs=0)
                            if attrNsName[1] == '*':
                                for attr in childNode.getAttributeDict().keys():
                                    if attr[0] == attrNsName[0]:
                                        if attrNodeList == []:
                                            attrNsNameFirst = attrNsName
                                        attrNodeList.append (childNode)
                                        stepChildList.append (childNode.getAttribute(attr))
                            elif childNode.hasAttribute(attrNsName):
                                if attrNodeList == []:
                                    attrNsNameFirst = attrNsName
                                attrNodeList.append (childNode)
                                stepChildList.append (childNode.getAttribute(attrNsName))
                    childList = stepChildList
                else:
                    nsLocalName = namespaceRef.qName2NsName (localStep, useDefaultNs=useDefaultNs)
                    if descendantOrSelf:
                        descendantOrSelf = 0
                        if localStep == "*":
                            stepChildList = self.getElementsByTagName()
                        else:
                            stepChildList = self.getElementsByTagName(nsLocalName)
                    else:
                        for childNode in childList:
                            if localStep == "*":
                                stepChildList.extend (childNode.getChildren())
                            else:
                                stepChildList.extend (childNode.getChildrenNS(nsLocalName[0], nsLocalName[1]))
                    childList = stepChildList
            # filter duplicated childs
            for child in childList:
                try:
                    childKey = child.element
                except:
                    childKey = child
                if not completeChildDict.has_key(childKey):
                    completeChildList.append(child)
                    completeChildDict[childKey] = 1
        return completeChildList, attrNodeList, attrNsNameFirst


    ###############################################################
    # PRIVATE methods
    ###############################################################

    def __createElement (self, tupleOrLocalName, attributeDict):
        """Create a new ElementWrapper object.
        
        Input parameter:
            tupleOrLocalName: tuple '(namespace, localName)' or 'localName' if no namespace is used
            attributeDict:    dictionary which contains the attributes and their values of the element node to be created
        Returns the created ElementWrapper object
        """
        childElementWrapper = self.treeWrapper.createElement (tupleOrLocalName, attributeDict, self.curNs[:]) # TODO: when to be adapted???)
        childElementWrapper.element.xmlIfExtSetParentNode(self.element)
        return childElementWrapper


    def __updateAttrMaxLengthDict (self, attrMaxLengthDict):
        """Update dictionary which contains the maximum length of node attributes.
        
        Used for pretty print to align the attributes of child nodes.
        attrMaxLengthDict is in/out parameter.
        """
        for attrName, attrValue in self.getAttributeDict().items():
            attrLength = len(attrValue)
            if not attrMaxLengthDict.has_key(attrName):
                attrMaxLengthDict[attrName] = attrLength
            else:
                attrMaxLengthDict[attrName] = max(attrMaxLengthDict[attrName], attrLength)


    def __clearChildrenCache (self, childNsName=None):
        """Clear children cache.
        """
        if self.__useCaching():
            if childNsName != None:
                if self.__childrenCache.has_key(childNsName):
                    del self.__childrenCache[childNsName]
                if self.__firstChildCache.has_key(childNsName):
                    del self.__firstChildCache[childNsName]
            else:
                self.__childrenCache.clear()
                self.__firstChildCache.clear()
                

    def __useCaching(self):
        return self.treeWrapper.useCaching()
    

