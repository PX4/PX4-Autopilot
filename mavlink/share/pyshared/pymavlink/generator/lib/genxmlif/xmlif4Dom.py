#
# genxmlif, Release 0.9.0
# file: xmlif4Dom.py
#
# XML interface class to the 4DOM library
#
# history:
# 2005-04-25 rl   created
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

import urllib
from xml.dom.ext.reader.Sax2   import Reader, XmlDomGenerator
from xml.sax._exceptions       import SAXParseException
from ..genxmlif                  import XMLIF_4DOM, GenXmlIfError
from xmlifUtils                import convertToAbsUrl
from xmlifDom                  import XmlInterfaceDom, XmlIfBuilderExtensionDom, InternalDomTreeWrapper, InternalDomElementWrapper


class XmlInterface4Dom (XmlInterfaceDom):
    #####################################################
    # for description of the interface methods see xmlifbase.py
    #####################################################

    def __init__ (self, verbose, useCaching, processXInclude):
        XmlInterfaceDom.__init__ (self, verbose, useCaching, processXInclude)
        self.xmlIfType = XMLIF_4DOM
        if self.verbose:
            print "Using 4Dom interface module..."


    def parse (self, file, baseUrl="", internalOwnerDoc=None):
        absUrl = convertToAbsUrl (file, baseUrl)
        fp     = urllib.urlopen (absUrl)
        return self._parseStream (fp, file, absUrl, internalOwnerDoc)


    def parseString (self, text, baseUrl="", internalOwnerDoc=None):
        import cStringIO
        fp = cStringIO.StringIO(text)
        absUrl = convertToAbsUrl ("", baseUrl)
        return self._parseStream (fp, "", absUrl, internalOwnerDoc)


    def _parseStream (self, fp, file, absUrl, internalOwnerDoc):
        reader = Reader(validate=0, keepAllWs=0, catName=None, 
                        saxHandlerClass=ExtXmlDomGenerator, parser=None)
        reader.handler.extinit(file, absUrl, reader.parser, self)
        if internalOwnerDoc != None: 
            ownerDoc = internalOwnerDoc.document
        else:
            ownerDoc = None
        try:
            tree = reader.fromStream(fp, ownerDoc)
            fp.close()
        except SAXParseException, errInst:
            fp.close()
            raise GenXmlIfError, "%s: SAXParseException: %s" %(file, str(errInst))

        treeWrapper = reader.handler.treeWrapper
        
        # XInclude support
        if self.processXInclude:
            if internalOwnerDoc == None: 
                internalOwnerDoc = treeWrapper.getTree()
            self.xInclude (treeWrapper.getRootNode(), absUrl, internalOwnerDoc)
            
        return treeWrapper


###################################################
# Extended DOM generator class derived from XmlDomGenerator
# extended to store related line numbers, file/URL names and 
# defined namespaces in the node object

class ExtXmlDomGenerator(XmlDomGenerator, XmlIfBuilderExtensionDom):
    def __init__(self, keepAllWs=0):
        XmlDomGenerator.__init__(self, keepAllWs)
        self.treeWrapper = None


    def extinit (self, filePath, absUrl, parser, xmlIf):
        self.filePath = filePath
        self.absUrl = absUrl
        self.parser = parser
        self.xmlIf = xmlIf


    def startElement(self, name, attribs):
        XmlDomGenerator.startElement(self, name, attribs)

        if not self.treeWrapper:
            self.treeWrapper = self.xmlIf.treeWrapperClass(self, InternalDomTreeWrapper(self._rootNode), self.xmlIf.useCaching)
            XmlIfBuilderExtensionDom.__init__(self, self.filePath, self.absUrl, self.treeWrapper, self.xmlIf.elementWrapperClass)

        curNode = self._nodeStack[-1]
        internal4DomElementWrapper = InternalDomElementWrapper(curNode, self.treeWrapper.getTree())
        curNs = self._namespaces.items()
        try:
            curNs.remove( (None,None) )
        except:
            pass

        XmlIfBuilderExtensionDom.startElementHandler (self, internal4DomElementWrapper, self.parser.getLineNumber(), curNs)


    def endElement(self, name):
        curNode = self._nodeStack[-1]
        XmlIfBuilderExtensionDom.endElementHandler (self, curNode.xmlIfExtInternalWrapper, self.parser.getLineNumber())
        XmlDomGenerator.endElement(self, name)
