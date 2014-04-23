#
# minixsv, Release 0.9.0
# file: pyxsval.py
#
# API for XML schema validator
#
# history:
# 2004-09-09 rl   created
# 2004-09-29 rl   adapted to re-designed XML interface classes,
#                 ErrorHandler separated, URL processing added, some bugs fixed
# 2004-10-07 rl   Validator classes extracted into separate files
# 2004-10-12 rl   API re-worked, XML text processing added
# 2007-05-15 rl   Handling of several schema files added,
#                 schema references in the input file have now priority (always used if available!)
# 2008-08-01 rl   New optional parameter 'useCaching=1' and 'processXInclude=1' to XsValidator class added
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

__all__ = [
    # public symbols
    "addUserSpecXmlIfClass",
    "parseAndValidate",
    "parseAndValidateString",
    "parseAndValidateXmlInput",
    "parseAndValidateXmlInputString",
    "parseAndValidateXmlSchema",
    "parseAndValidateXmlSchemaString",
    "XsValidator",
    ]


import string
from .. import genxmlif
from ..minixsv           import *
from xsvalErrorHandler import ErrorHandler
from xsvalXmlIf        import XsvXmlElementWrapper
from xsvalBase         import XsValBase
from xsvalSchema       import XsValSchema


__author__  = "Roland Leuthe <roland@leuthe-net.de>"
__date__    = "08. August 2008"
__version__ = "0.9.0"


_XS_VAL_DEFAULT_ERROR_LIMIT = 20

rulesTreeWrapper = None   # XML tree cache for "XMLSchema.xsd"


########################################
# retrieve version of minixsv
#
def getVersion ():
    return __version__


########################################
# access function for adding a user specific XML interface class
#
def addUserSpecXmlIfClass (xmlIfKey, factory):
    if not _xmlIfDict.has_key(xmlIfKey):
        _xmlIfDict[xmlIfKey] = factory
    else:
        raise KeyError, "xmlIfKey %s already implemented!" %(xmlIfKey)


########################################
# convenience function for validating
# 1. XML schema file
# 2. XML input file
# If xsdFile is specified, it will ONLY be used for validation if no schema file 
# is specified in the input file
# If xsdFile=None, the schemaLocation attribute is expected in the root tag of the XML input file
#
def parseAndValidate (inputFile, xsdFile=None, **kw):
    return parseAndValidateXmlInput (inputFile, xsdFile, 1, **kw)


########################################
# convenience function for validating
# 1. text string containing the XML schema
# 2. text string containing the XML input
# If xsdText is given, it will ONLY be used for validation if no schema file
# is specified in the input text
# If xsdText=None, the schemaLocation attribute is expected in the root tag of the XML input
#
def parseAndValidateString (inputText, xsdText=None, **kw):
    return parseAndValidateXmlInputString (inputText, xsdText, 1, **kw)


########################################
# factory for validating
# 1. XML schema file (only if validateSchema=1)
# 2. XML input file
# If xsdFile is specified, it will ONLY be used for validation if no schema file 
# is specified in the input file
# If xsdFile=None, the schemaLocation attribute is expected in the root tag of the XML input file
#
def parseAndValidateXmlInput (inputFile, xsdFile=None, validateSchema=0, **kw):
    xsValidator = XsValidator (**kw)
    # parse XML input file
    inputTreeWrapper = xsValidator.parse (inputFile)
    # validate XML input file
    return xsValidator.validateXmlInput (inputFile, inputTreeWrapper, xsdFile, validateSchema)


########################################
# factory for validating
# 1. text string containing the XML schema (only if validateSchema=1)
# 2. text string containing the XML input
# If xsdText is given, it will ONLY be used for validation if no schema file
# is specified in the input text
# If xsdText=None, the schemaLocation attribute is expected in the root tag of the XML input
#
def parseAndValidateXmlInputString (inputText, xsdText=None, baseUrl="", validateSchema=0, **kw):
    xsValidator = XsValidator (**kw)
    # parse XML input text string
    inputTreeWrapper = xsValidator.parseString (inputText, baseUrl)
    # validate XML input text string
    return xsValidator.validateXmlInputString (inputTreeWrapper, xsdText, validateSchema)


########################################
# factory for validating only given XML schema file
#
def parseAndValidateXmlSchema (xsdFile, **kw):
    xsValidator = XsValidator (**kw)
    # parse XML schema file
    xsdTreeWrapper = xsValidator.parse (xsdFile)
    # validate XML schema file
    return xsValidator.validateXmlSchema (xsdFile, xsdTreeWrapper)


########################################
# factory for validating only given XML schema file
#
def parseAndValidateXmlSchemaString (xsdText, **kw):
    xsValidator = XsValidator (**kw)
    # parse XML schema
    xsdTreeWrapper = xsValidator.parseString (xsdText)
    # validate XML schema
    return xsValidator.validateXmlSchema ("", xsdTreeWrapper)


########################################
# XML schema validator class
#
class XsValidator:
    def __init__(self, xmlIfClass=XMLIF_MINIDOM,
                 elementWrapperClass=XsvXmlElementWrapper,
                 warningProc=IGNORE_WARNINGS, errorLimit=_XS_VAL_DEFAULT_ERROR_LIMIT, 
                 verbose=0, useCaching=1, processXInclude=1):

        self.warningProc    = warningProc
        self.errorLimit     = errorLimit
        self.verbose        = verbose

        # select XML interface class
        self.xmlIf = _xmlIfDict[xmlIfClass](verbose, useCaching, processXInclude)
        self.xmlIf.setElementWrapperClass (elementWrapperClass)

        # create error handler
        self.errorHandler  = ErrorHandler (errorLimit, warningProc, verbose)

        self.schemaDependancyList = []


    ########################################
    # retrieve current version
    #
    def getVersion (self):
        return __version__
        

    ########################################
    # parse XML file
    # 'file' may be a filepath or an URI
    #
    def parse (self, file, baseUrl="", ownerDoc=None):
        self._verbosePrint ("Parsing %s..." %(file))
        return self.xmlIf.parse(file, baseUrl, ownerDoc)


    ########################################
    # parse text string containing XML
    #
    def parseString (self, text, baseUrl=""):
        self._verbosePrint ("Parsing XML text string...")
        return self.xmlIf.parseString(text, baseUrl)


    ########################################
    # validate XML input
    #
    def validateXmlInput (self, xmlInputFile, inputTreeWrapper, xsdFile=None, validateSchema=0):
        # if the input file contains schema references => use these
        xsdTreeWrapperList = self._readReferencedXsdFiles(inputTreeWrapper, validateSchema)
        if xsdTreeWrapperList == []:
            # if the input file doesn't contain schema references => use given xsdFile
            if xsdFile != None:
                xsdTreeWrapper = self.parse (xsdFile)
                xsdTreeWrapperList.append(xsdTreeWrapper)
                # validate XML schema file if requested
                if validateSchema:
                    self.validateXmlSchema (xsdFile, xsdTreeWrapper)
            else:
                self.errorHandler.raiseError ("No schema file specified!")

        self._validateXmlInput (xmlInputFile, inputTreeWrapper, xsdTreeWrapperList)
        for xsdTreeWrapper in xsdTreeWrapperList:
            xsdTreeWrapper.unlink()
        return inputTreeWrapper

    ########################################
    # validate XML input
    #
    def validateXmlInputString (self, inputTreeWrapper, xsdText=None, validateSchema=0):
        # if the input file contains schema references => use these
        xsdTreeWrapperList = self._readReferencedXsdFiles(inputTreeWrapper, validateSchema)
        if xsdTreeWrapperList == []:
            # if the input file doesn't contain schema references => use given xsdText
            if xsdText != None:
                xsdFile = "schema text"
                xsdTreeWrapper = self.parseString (xsdText)
                xsdTreeWrapperList.append(xsdTreeWrapper)
                # validate XML schema file if requested
                if validateSchema:
                    self.validateXmlSchema (xsdFile, xsdTreeWrapper)
            else:
                self.errorHandler.raiseError ("No schema specified!")

        self._validateXmlInput ("input text", inputTreeWrapper, xsdTreeWrapperList)
        for xsdTreeWrapper in xsdTreeWrapperList:
            xsdTreeWrapper.unlink()
        return inputTreeWrapper


    ########################################
    # validate XML schema separately
    #
    def validateXmlSchema (self, xsdFile, xsdTreeWrapper):
        # parse minixsv internal schema
        global rulesTreeWrapper
        if rulesTreeWrapper == None:
            rulesTreeWrapper = self.parse(os.path.join (MINIXSV_DIR, "XMLSchema.xsd"))

        self._verbosePrint ("Validating %s..." %(xsdFile))
        xsvGivenXsdFile = XsValSchema (self.xmlIf, self.errorHandler, self.verbose)
        xsvGivenXsdFile.validate(xsdTreeWrapper, [rulesTreeWrapper,])
        self.schemaDependancyList.append (xsdFile)
        self.schemaDependancyList.extend (xsvGivenXsdFile.xsdIncludeDict.keys())
        xsvGivenXsdFile.unlink()
        self.errorHandler.flushOutput()
        return xsdTreeWrapper


    ########################################
    # validate XML input tree and xsd tree if requested
    #
    def _validateXmlInput (self, xmlInputFile, inputTreeWrapper, xsdTreeWrapperList):
        self._verbosePrint ("Validating %s..." %(xmlInputFile))
        xsvInputFile = XsValBase (self.xmlIf, self.errorHandler, self.verbose)
        xsvInputFile.validate(inputTreeWrapper, xsdTreeWrapperList)
        xsvInputFile.unlink()
        self.errorHandler.flushOutput()


    ########################################
    # retrieve XML schema location from XML input tree
    #
    def _readReferencedXsdFiles (self, inputTreeWrapper, validateSchema):
        xsdTreeWrapperList = []
        # a schemaLocation attribute is expected in the root tag of the XML input file
        xsdFileList = self._retrieveReferencedXsdFiles (inputTreeWrapper)
        for namespace, xsdFile in xsdFileList:
            try:
                xsdTreeWrapper = self.parse (xsdFile, inputTreeWrapper.getRootNode().getAbsUrl())
            except IOError, e:
                if e.errno == 2: # catch IOError: No such file or directory
                    self.errorHandler.raiseError ("XML schema file %s not found!" %(xsdFile), inputTreeWrapper.getRootNode())
                else:
                    raise IOError(e.errno, e.strerror, e.filename)

            xsdTreeWrapperList.append(xsdTreeWrapper)
            # validate XML schema file if requested
            if validateSchema:
                self.validateXmlSchema (xsdFile, xsdTreeWrapper)

            if namespace != xsdTreeWrapper.getRootNode().getAttributeOrDefault("targetNamespace", None):
                self.errorHandler.raiseError ("Namespace of 'schemaLocation' attribute doesn't match target namespace of %s!" %(xsdFile), inputTreeWrapper.getRootNode())
            
        return xsdTreeWrapperList


    ########################################
    # retrieve XML schema location from XML input tree
    #
    def _retrieveReferencedXsdFiles (self, inputTreeWrapper):
        # a schemaLocation attribute is expected in the root tag of the XML input file
        inputRootNode = inputTreeWrapper.getRootNode()
        xsdFileList = []

        if inputRootNode.hasAttribute((XSI_NAMESPACE, "schemaLocation")):
            attributeValue = inputRootNode.getAttribute((XSI_NAMESPACE, "schemaLocation"))
            attrValList = string.split(attributeValue)
            if len(attrValList) % 2 == 0:
                for i in range(0, len(attrValList), 2):
                    xsdFileList.append((attrValList[i], attrValList[i+1]))
            else:
                self.errorHandler.raiseError ("'schemaLocation' attribute must have even number of URIs (pairs of namespace and xsdFile)!")

        if inputRootNode.hasAttribute((XSI_NAMESPACE, "noNamespaceSchemaLocation")):
            attributeValue = inputRootNode.getAttribute((XSI_NAMESPACE, "noNamespaceSchemaLocation"))
            attrValList = string.split(attributeValue)
            for attrVal in attrValList:
                xsdFileList.append ((None, attrVal))

        return xsdFileList

    ########################################
    # print if verbose flag is set
    #
    def _verbosePrint (self, text):
        if self.verbose:
            print text


########################################
# factory functions for enabling the selected XML interface class
#
def _interfaceFactoryMinidom (verbose, useCaching, processXInclude):
    return genxmlif.chooseXmlIf(genxmlif.XMLIF_MINIDOM, verbose, useCaching, processXInclude)

def _interfaceFactory4Dom (verbose, useCaching, processXInclude):
    return genxmlif.chooseXmlIf(genxmlif.XMLIF_4DOM, verbose, useCaching, processXInclude)

def _interfaceFactoryElementTree (verbose, useCaching, processXInclude):
    return genxmlif.chooseXmlIf(genxmlif.XMLIF_ELEMENTTREE, verbose, useCaching, processXInclude)


_xmlIfDict = {XMLIF_MINIDOM    :_interfaceFactoryMinidom,
              XMLIF_4DOM       :_interfaceFactory4Dom,
              XMLIF_ELEMENTTREE:_interfaceFactoryElementTree}

