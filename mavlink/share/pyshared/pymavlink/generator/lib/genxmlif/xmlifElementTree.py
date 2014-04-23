#
# genxmlif, Release 0.9.0
# file: xmlifElementTree.py
#
# XML interface class to elementtree toolkit by Fredrik Lundh
#
# history:
# 2005-04-25 rl   created
# 2007-05-25 rl   performance optimization (caching) added, some bugfixes
# 2007-06-29 rl   complete re-design, ElementExtension class introduced
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

import sys
import string
import urllib
from xml.dom           import EMPTY_NAMESPACE, XMLNS_NAMESPACE
from xml.parsers.expat import ExpatError
# from version 2.5 on the elementtree module is part of the standard python distribution
if sys.version_info[:2] >= (2,5):
    from xml.etree.ElementTree      import ElementTree, _ElementInterface, XMLTreeBuilder, TreeBuilder
    from xml.etree import ElementInclude 
else:
    from elementtree.ElementTree    import ElementTree, _ElementInterface, XMLTreeBuilder, TreeBuilder
    from elementtree import ElementInclude 
from ..genxmlif                   import XMLIF_ELEMENTTREE, GenXmlIfError
from xmlifUtils                 import convertToAbsUrl, processWhitespaceAction, collapseString, toClarkQName, splitQName
from xmlifBase                  import XmlIfBuilderExtensionBase
from xmlifApi                   import XmlInterfaceBase

#########################################################
# Derived interface class for elementtree toolkit

class XmlInterfaceElementTree (XmlInterfaceBase):
    #####################################################
    # for description of the interface methods see xmlifbase.py
    #####################################################

    def __init__ (self, verbose, useCaching, processXInclude):
        XmlInterfaceBase.__init__ (self, verbose, useCaching, processXInclude)
        self.xmlIfType = XMLIF_ELEMENTTREE
        if self.verbose:
            print "Using elementtree interface module..."


    def createXmlTree (self, namespace, xmlRootTagName, attributeDict={}, publicId=None, systemId=None):
        rootNode = ElementExtension(toClarkQName(xmlRootTagName), attributeDict)
        rootNode.xmlIfExtSetParentNode(None)
        treeWrapper = self.treeWrapperClass(self, ElementTreeExtension(rootNode), self.useCaching)
        rootNodeWrapper = self.elementWrapperClass (rootNode, treeWrapper, []) # TODO: namespace handling
        return treeWrapper


    def parse (self, file, baseUrl="", ownerDoc=None):
        absUrl = convertToAbsUrl (file, baseUrl)
        fp     = urllib.urlopen (absUrl)
        try:
            tree        = ElementTreeExtension()
            treeWrapper = self.treeWrapperClass(self, tree, self.useCaching)
            parser = ExtXMLTreeBuilder(file, absUrl, self, treeWrapper)
            treeWrapper.getTree().parse(fp, parser)
            fp.close()
            
            # XInclude support
            if self.processXInclude:
                loaderInst = ExtXIncludeLoader (self.parse, absUrl, ownerDoc)
                try:
                    ElementInclude.include(treeWrapper.getTree().getroot(), loaderInst.loader)
                except IOError, errInst:
                    raise GenXmlIfError, "%s: IOError: %s" %(file, str(errInst))
            
        except ExpatError, errstr:
            fp.close()
            raise GenXmlIfError, "%s: ExpatError: %s" %(file, str(errstr))
        except ElementInclude.FatalIncludeError, errInst:
            fp.close()
            raise GenXmlIfError, "%s: XIncludeError: %s" %(file, str(errInst))
            
        return treeWrapper


    def parseString (self, text, baseUrl="", ownerDoc=None):
        absUrl = convertToAbsUrl ("", baseUrl)
        tree        = ElementTreeExtension()
        treeWrapper = self.treeWrapperClass(self, tree, self.useCaching)
        parser = ExtXMLTreeBuilder("", absUrl, self, treeWrapper)
        parser.feed(text)
        treeWrapper.getTree()._setroot(parser.close())

        # XInclude support
        if self.processXInclude:
            loaderInst = ExtXIncludeLoader (self.parse, absUrl, ownerDoc)
            ElementInclude.include(treeWrapper.getTree().getroot(), loaderInst.loader)

        return treeWrapper


#########################################################
# Extension (derived) class for ElementTree class

class ElementTreeExtension (ElementTree):

    def xmlIfExtGetRootNode (self):
        return self.getroot()


    def xmlIfExtCreateElement (self, nsName, attributeDict, curNs):
        clarkQName = toClarkQName(nsName)
        return ElementExtension (clarkQName, attributeDict)


    def xmlIfExtCloneTree (self, rootElementCopy):
        return self.__class__(element=rootElementCopy)
        

#########################################################
# Wrapper class for Element class

class ElementExtension (_ElementInterface):

    def __init__ (self, xmlRootTagName, attributeDict):
        _ElementInterface.__init__(self, xmlRootTagName, attributeDict)


    def xmlIfExtUnlink (self):
        self.xmlIfExtElementWrapper = None
        self.__xmlIfExtParentElement = None
        

    def xmlIfExtCloneNode (self):
        nodeCopy = self.__class__(self.tag, self.attrib.copy())
        nodeCopy.text = self.text
        nodeCopy.tail = self.tail
        return nodeCopy
    

    def xmlIfExtGetTagName (self):
        return self.tag


    def xmlIfExtGetNamespaceURI (self):
        prefix, localName = splitQName(self.tag)
        return prefix


    def xmlIfExtGetParentNode (self):
        return self.__xmlIfExtParentElement


    def xmlIfExtSetParentNode (self, parentElement):
        self.__xmlIfExtParentElement = parentElement
    

    def xmlIfExtGetChildren (self, filterTag=None):
        if filterTag == None:
            return self.getchildren()
        else:
            clarkFilterTag = toClarkQName(filterTag)
            return self.findall(clarkFilterTag)


    def xmlIfExtGetFirstChild (self, filterTag=None):
        # replace base method (performance optimized)
        if filterTag == None:
            children = self.getchildren()
            if children != []:
                element = children[0]
            else:
                element = None
        else:
            clarkFilterTag = toClarkQName(filterTag)
            element = self.find(clarkFilterTag)

        return element


    def xmlIfExtGetElementsByTagName (self, filterTag=(None,None)):
        clarkFilterTag = toClarkQName(filterTag)
        descendants = []
        for node in self.xmlIfExtGetChildren():
            descendants.extend(node.getiterator(clarkFilterTag))
        return descendants


    def xmlIfExtGetIterator (self, filterTag=(None,None)):
        clarkFilterTag = toClarkQName(filterTag)
        return self.getiterator (clarkFilterTag)

    
    def xmlIfExtAppendChild (self, childElement):
        self.append (childElement)
        childElement.xmlIfExtSetParentNode(self)


    def xmlIfExtInsertBefore (self, childElement, refChildElement):
        self.insert (self.getchildren().index(refChildElement), childElement)
        childElement.xmlIfExtSetParentNode(self)


    def xmlIfExtRemoveChild (self, childElement):
        self.remove (childElement)


    def xmlIfExtInsertSubtree (self, refChildElement, subTree, insertSubTreeRootNode):
        if refChildElement != None:
            insertIndex = self.getchildren().index (refChildElement)
        else:
            insertIndex = 0
        if insertSubTreeRootNode:
            elementList = [subTree.xmlIfExtGetRootNode(),]
        else:
            elementList = subTree.xmlIfExtGetRootNode().xmlIfExtGetChildren()
        elementList.reverse()
        for element in elementList:
            self.insert (insertIndex, element)
            element.xmlIfExtSetParentNode(self)


    def xmlIfExtGetAttributeDict (self):
        attrDict = {}
        for attrName, attrValue in self.attrib.items():
            namespaceEndIndex = string.find (attrName, '}')
            if namespaceEndIndex != -1:
                attrName = (attrName[1:namespaceEndIndex], attrName[namespaceEndIndex+1:])
            else:
                attrName = (EMPTY_NAMESPACE, attrName)
            attrDict[attrName] = attrValue
        return attrDict


    def xmlIfExtGetAttribute (self, tupleOrAttrName):
        clarkQName = toClarkQName(tupleOrAttrName)
        if self.attrib.has_key(clarkQName):
            return self.attrib[clarkQName]
        else:
            return None


    def xmlIfExtSetAttribute (self, tupleOrAttrName, attributeValue, curNs):
        self.attrib[toClarkQName(tupleOrAttrName)] = attributeValue


    def xmlIfExtRemoveAttribute (self, tupleOrAttrName):
        clarkQName = toClarkQName(tupleOrAttrName)
        if self.attrib.has_key(clarkQName):
            del self.attrib[clarkQName]


    def xmlIfExtGetElementValueFragments (self, ignoreEmtpyStringFragments):
        elementValueList = []
        if self.text != None:
            elementValueList.append(self.text)
        for child in self.getchildren():
            if child.tail != None:
                elementValueList.append(child.tail)
        if ignoreEmtpyStringFragments:
            elementValueList = filter (lambda s: collapseString(s) != "", elementValueList)
        if elementValueList == []:
            elementValueList = ["",]
        return elementValueList


    def xmlIfExtGetElementText (self):
        if self.text != None:
            return self.text
        else:
            return ""

    
    def xmlIfExtGetElementTailText (self):
        if self.tail != None:
            return self.tail
        else:
            return ""
    

    def xmlIfExtSetElementValue (self, elementValue):
        self.text = elementValue
        for child in self.getchildren():
            child.tail = None
            

    def xmlIfExtProcessWsElementValue (self, wsAction):
        noOfTextFragments = reduce(lambda sum, child: sum + (child.tail != None), self.getchildren(), 0)
        noOfTextFragments += (self.text != None)
                
        rstrip = 0
        lstrip = 1
        if self.text != None:
            if noOfTextFragments == 1:
                rstrip = 1
            self.text = processWhitespaceAction (self.text, wsAction, lstrip, rstrip)
            noOfTextFragments -= 1
            lstrip = 0
        for child in self.getchildren():
            if child.tail != None:
                if noOfTextFragments == 1:
                    rstrip = 1
                child.tail = processWhitespaceAction (child.tail, wsAction, lstrip, rstrip)
                noOfTextFragments -= 1
                lstrip = 0


###################################################
# Element tree builder class derived from XMLTreeBuilder
# extended to store related line numbers in the Element object

class ExtXMLTreeBuilder (XMLTreeBuilder, XmlIfBuilderExtensionBase):
    def __init__(self, filePath, absUrl, xmlIf, treeWrapper):
        XMLTreeBuilder.__init__(self, target=TreeBuilder(element_factory=ElementExtension))
        self._parser.StartNamespaceDeclHandler = self._start_ns
        self._parser.EndNamespaceDeclHandler = self._end_ns
        self.namespaces = []
        XmlIfBuilderExtensionBase.__init__(self, filePath, absUrl, treeWrapper, xmlIf.elementWrapperClass)

    def _start(self, tag, attrib_in):
        elem = XMLTreeBuilder._start(self, tag, attrib_in)
        self.start(elem)

    def _start_list(self, tag, attrib_in):
        elem = XMLTreeBuilder._start_list(self, tag, attrib_in)
        self.start(elem, attrib_in)

    def _end(self, tag):
        elem = XMLTreeBuilder._end(self, tag)
        self.end(elem)

    def _start_ns(self, prefix, value):
        self.namespaces.insert(0, (prefix, value))

    def _end_ns(self, prefix):
        assert self.namespaces.pop(0)[0] == prefix, "implementation confused"


    def start(self, element, attributes):
        # bugfix for missing start '{'
        for i in range (0, len(attributes), 2):
            attrName = attributes[i]
            namespaceEndIndex = string.find (attrName, '}')
            if namespaceEndIndex != -1 and attrName[0] != "{":
                attributes[i] = '{' + attributes[i]
        # bugfix end

        XmlIfBuilderExtensionBase.startElementHandler (self, element, self._parser.ErrorLineNumber, self.namespaces[:], attributes)
        if len(self._target._elem) > 1:
            element.xmlIfExtSetParentNode (self._target._elem[-2])
        else:
            for namespace in self.namespaces:
                if namespace[1] != None:
                    element.xmlIfExtElementWrapper.setAttribute((XMLNS_NAMESPACE, namespace[0]), namespace[1])


    def end(self, element):
        XmlIfBuilderExtensionBase.endElementHandler (self, element, self._parser.ErrorLineNumber)


###################################################
# XInclude loader
# 

class ExtXIncludeLoader:

    def __init__(self, parser, baseUrl, ownerDoc):
        self.parser = parser
        self.baseUrl = baseUrl
        self.ownerDoc = ownerDoc
    
    def loader(self, href, parse, encoding=None):
        if parse == "xml":
            data = self.parser(href, self.baseUrl, self.ownerDoc).getTree().getroot()
        else:
            absUrl = convertToAbsUrl (href, self.baseUrl)
            fp     = urllib.urlopen (absUrl)
            data = fp.read()
            if encoding:
                data = data.decode(encoding)
            fp.close()
        return data
