#
# minixsv, Release 0.9.0
# file: xsvalBase.py
#
# XML schema validator base class
#
# history:
# 2004-10-07 rl   created
# 2006-08-18 rl   W3C testsuite passed for supported features
# 2007-06-14 rl   Features for release 0.8 added, several bugs fixed
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

import string
import copy
from ..minixsv             import *
from ..genxmlif.xmlifUtils import collapseString, convertToAbsUrl, NsNameTupleFactory, NsNameTuple
from xsvalSimpleTypes    import XsSimpleTypeVal, SimpleTypeError


wxsdTree = None
wxsdLookupDict = {}


##########################################################
#  Validator class for validating one input file against one XML schema file

class XsValBase:

    def __init__(self, xmlIf, errorHandler, verbose):
        self.xmlIf         = xmlIf
        self.errorHandler  = errorHandler
        self.verbose       = verbose

        self._raiseError   = self.errorHandler.raiseError
        self._addError     = self.errorHandler.addError
        self._addWarning   = self.errorHandler.addWarning
        self._addInfo      = self.errorHandler.addInfo

        self.checkKeyrefList = []


    def unlink(self):
        self.simpleTypeVal.unlink()
        

    ########################################
    # validate inputTree against xsdTree
    #
    def validate (self, inputTree, xsdTreeList):
        self.inputTree = inputTree

        self.inputRoot   = self.inputTree.getRootNode()

        self.inputNsURI  = self.inputRoot.getNamespaceURI()
        self.inputNsPrefix = self.inputRoot.getNsPrefix(self.inputRoot.getNsName())
        if self.inputNsPrefix != None:
            self.inputNsPrefixString = "%s:" %(self.inputNsPrefix)
        else:
            self.inputNsPrefixString = ""

        # initialise lookup dictionary
        global wxsdLookupDict
        if wxsdLookupDict == {}:
            wxsdLookupDict = {"ElementDict":{}, "TypeDict":{}, "GroupDict":{},
                              "AttrGroupDict":{}, "AttributeDict":{}, "IdentityConstrDict":{}}
            self._importWellknownSchemas(wxsdLookupDict)
   
        self.xsdLookupDict = {"ElementDict":        wxsdLookupDict["ElementDict"].copy(), 
                              "TypeDict":           wxsdLookupDict["TypeDict"].copy(), 
                              "GroupDict":          wxsdLookupDict["GroupDict"].copy(),
                              "AttrGroupDict":      wxsdLookupDict["AttrGroupDict"].copy(), 
                              "AttributeDict":      wxsdLookupDict["AttributeDict"].copy(), 
                              "IdentityConstrDict": wxsdLookupDict["IdentityConstrDict"].copy()}
        self.xsdElementDict        = self.xsdLookupDict["ElementDict"]
        self.xsdTypeDict           = self.xsdLookupDict["TypeDict"]
        self.xsdGroupDict          = self.xsdLookupDict["GroupDict"]
        self.xsdAttrGroupDict      = self.xsdLookupDict["AttrGroupDict"]
        self.xsdAttributeDict      = self.xsdLookupDict["AttributeDict"]
        self.xsdIdentityConstrDict = self.xsdLookupDict["IdentityConstrDict"]

        self.xsdIdDict = {}
        self.xsdIdRefDict = {}
        self.idAttributeForType = None

        for xsdTree in xsdTreeList:
            xsdRoot     = xsdTree.getRootNode()

            # TODO: The following member may differ if several schema files are used!!
            self.xsdNsURI    = xsdRoot.getNamespaceURI()

            self.xsdIncludeDict = {xsdRoot.getAbsUrl():1,}
            if xsdRoot.getFilePath() != os.path.join (MINIXSV_DIR, "XMLSchema.xsd"):
                self._initInternalAttributes (xsdRoot)
                self._updateLookupTables(xsdRoot, self.xsdLookupDict)
            
                self._includeAndImport (xsdTree, xsdTree, self.xsdIncludeDict, self.xsdLookupDict)
            

        self.simpleTypeVal = XsSimpleTypeVal(self)

        inputRootNsName = self.inputRoot.getNsName()
        if self.xsdElementDict.has_key(inputRootNsName):
            # start recursive schema validation
            try:
                self._checkElementTag (self.xsdElementDict[inputRootNsName], self.inputRoot, (self.inputRoot,), 0)
            except TagException, errInst:
                self._addError (errInst.errstr, errInst.node, errInst.endTag)

            if not self.errorHandler.hasErrors():
                # validate IDREFs
                for idref in self.xsdIdRefDict.keys():
                    if not self.xsdIdDict.has_key(idref):
                        self._addError ("There is no ID/IDREF binding for IDREF %s" %repr(idref), self.xsdIdRefDict[idref])

                # validate keyrefs
                for inputElement, keyrefNode in self.checkKeyrefList:
                    self._checkKeyRefConstraint (keyrefNode, inputElement)
        else:
            self._raiseError ("Used root tag %s not found in schema file(s)!"
                              %repr(inputRootNsName), self.inputRoot)
        

    ########################################
    # include/import all files specified in the schema file
    # import well-known schemas
    #
    def _includeAndImport (self, baseTree, tree, includeDict, lookupDict):
        self._expandIncludes (baseTree, tree, includeDict, lookupDict)
        self._expandRedefines (baseTree, tree, includeDict, lookupDict)
        self._expandImports (baseTree, tree, includeDict, lookupDict)


    ########################################
    # expand include directives
    #
    def _expandIncludes (self, baseTree, tree, includeDict, lookupDict):
        rootNode = tree.getRootNode()
        namespaceURI  = rootNode.getNamespaceURI()
        for includeNode in rootNode.getChildrenNS(namespaceURI, "include"):
            includeUrl = includeNode.getAttribute("schemaLocation")
            expNamespace = rootNode.getAttributeOrDefault("targetNamespace", None)
            self._includeSchemaFile (baseTree, tree, includeNode, expNamespace, includeUrl, includeNode.getBaseUrl(), includeDict, lookupDict,
                                     adaptTargetNamespace=1)
            rootNode.removeChild (includeNode)


    ########################################
    # expand redefine directives
    #
    def _expandRedefines (self, baseTree, tree, includeDict, lookupDict):
        rootNode = tree.getRootNode()
        namespaceURI  = rootNode.getNamespaceURI()

        for redefineNode in rootNode.getChildrenNS(namespaceURI, "redefine"):
            redefineUrl = redefineNode.getAttribute("schemaLocation")
            expNamespace = rootNode.getAttributeOrDefault("targetNamespace", None)
            self._includeSchemaFile (baseTree, tree, redefineNode, expNamespace, redefineUrl, redefineNode.getBaseUrl(), includeDict, lookupDict,
                                     adaptTargetNamespace=1)

            # fill lookup tables with redefined definitions
            for childNode in redefineNode.getChildren():
                redefineNode.removeChild(childNode)
                rootNode.insertBefore(childNode, redefineNode)

                if childNode.getLocalName() in ("complexType", "simpleType"):
                    xsdDict = self.xsdLookupDict["TypeDict"]
                elif childNode.getLocalName() in ("attributeGroup"):
                    xsdDict = self.xsdLookupDict["AttrGroupDict"]
                elif childNode.getLocalName() in ("group"):
                    xsdDict = self.xsdLookupDict["GroupDict"]
                elif childNode.getLocalName() in ("annotation"):
                    continue
                else:
                    self._addError ("%s not allowed as child of 'redefine'!" %repr(childNode.getLocalName()), childNode)
                    continue

                redefType = NsNameTuple ( (expNamespace, childNode.getAttribute("name")) )
                if xsdDict.has_key(redefType):
                    orgRedefType = NsNameTuple( (expNamespace, redefType[1]+"__ORG") )
                    if not xsdDict.has_key(orgRedefType):
                        xsdDict[orgRedefType] = xsdDict[redefType]
#                    else:
#                        self._addError ("Duplicate component %s found within 'redefine'!" %repr(redefType), childNode)
                    xsdDict[redefType] = childNode
                else:
                    self._addError ("Type %s not found in imported schema file!" %(repr(redefType)), childNode)

                dummy, attrNodes, attrNsNameFirst = childNode.getXPathList (".//@base | .//@ref" % vars())
                for attrNode in attrNodes:
                    if attrNode.hasAttribute("base"):
                        attribute = "base"
                    elif attrNode.hasAttribute("ref"):
                        attribute = "ref"
                    if attrNode.getQNameAttribute(attribute) == redefType:
                        attrNode[attribute] = attrNode[attribute] + "__ORG"
                
            rootNode.removeChild (redefineNode)


    ########################################
    # expand import directives
    #
    def _expandImports (self, baseTree, tree, includeDict, lookupDict):
        rootNode = tree.getRootNode()
        namespaceURI  = rootNode.getNamespaceURI()

        for includeNode in rootNode.getChildrenNS(namespaceURI, "import"):
            expNamespace = includeNode.getAttributeOrDefault("namespace", None)
            if expNamespace == self._getTargetNamespace(includeNode):
                self._addError ("Target namespace and target namespace of imported schema must not be the same!",  includeNode)
                continue

            includeUrl = includeNode.getAttributeOrDefault("schemaLocation", None)
            if expNamespace != None and includeUrl == None:
                includeUrl = expNamespace + ".xsd"
            if includeUrl != None:            
                if expNamespace not in (XML_NAMESPACE, XSI_NAMESPACE):
                    self._includeSchemaFile (baseTree, tree, includeNode, expNamespace, includeUrl, includeNode.getBaseUrl(), includeDict, lookupDict)
            else:
                self._addError ("schemaLocation attribute for import directive missing!",  includeNode)
            rootNode.removeChild (includeNode)


    ########################################
    # import well-known schema files
    #
    def _importWellknownSchemas (self, lookupDict):
        global wxsdTree
        file = os.path.join (MINIXSV_DIR, "XMLSchema.xsd")
        wxsdTree = self.xmlIf.parse (file)
        self._initInternalAttributes (wxsdTree.getRootNode())
        self._updateLookupTables (wxsdTree.getRootNode(), lookupDict)

        for schemaFile in ("datatypes.xsd", "xml.xsd", "XMLSchema-instance.xsd"):
            file = os.path.join (MINIXSV_DIR, schemaFile)
            subTree = self._parseIncludeSchemaFile(wxsdTree, wxsdTree, None, file, None)
            self._updateLookupTables (subTree.getRootNode(), lookupDict)


    ########################################
    # include/import a schema file
    #
    def _includeSchemaFile (self, baseTree, tree, nextSibling, expNamespace, includeUrl, baseUrl, includeDict, lookupDict, 
                            adaptTargetNamespace=0):
        if includeUrl == None:
            self._raiseError ("Schema location attribute missing!", nextSibling)
        absUrl = convertToAbsUrl (includeUrl, baseUrl)
        if includeDict.has_key (absUrl):
            # file already included
            return

        if self.verbose:
            print "including %s..." %(includeUrl)
        rootNode = tree.getRootNode()

        subTree = self._parseIncludeSchemaFile(baseTree, tree, nextSibling, includeUrl, baseUrl)
        includeDict[absUrl] = 1

        stRootNode = subTree.getRootNode()
        if rootNode.getNsName() != stRootNode.getNsName():
            self._raiseError ("Root tag of file %s does not match!" %repr(includeUrl), nextSibling)

        if stRootNode.hasAttribute("targetNamespace"):
            if expNamespace != stRootNode["targetNamespace"]:
                self._raiseError ("Target namespace of file %s does not match!" %repr(includeUrl), nextSibling)
        else:
           if expNamespace != None: 
               if adaptTargetNamespace:
                    # if no target namespace is specified in the included file
                    # the target namespace of the parent file is taken
                    stRootNode["targetNamespace"] = expNamespace
                    for stDescNode in stRootNode.getIterator():
                        stDescNode.curNs.append((EMPTY_PREFIX,expNamespace))
               else:
                   self._raiseError ("Target namespace of file %s does not match!" %repr(includeUrl), nextSibling)
                    
        self._updateLookupTables (subTree.getRootNode(), lookupDict)
        self._includeAndImport (baseTree, subTree, includeDict, lookupDict)
        if includeUrl not in (r"C:\Program Files\Python24\Lib\site-packages\minixsv\xml.xsd",
                              r"C:\Program Files\Python24\Lib\site-packages\minixsv\XMLSchema.xsd",
                              r"C:\Program Files\Python24\Lib\site-packages\minixsv\XMLSchema-instance.xsd"):
            rootNode.insertSubtree (nextSibling, subTree, insertSubTreeRootNode=0)


    def _parseIncludeSchemaFile (self, baseTree, tree, nextSibling, includeUrl, baseUrl):
        # try to parse included schema file
        try:
            subTree = self.xmlIf.parse (includeUrl, baseUrl, baseTree.getTree())
            self._initInternalAttributes (subTree.getRootNode())
        except IOError, errInst:
            self._raiseError ("%s" %str(errInst), nextSibling)
        except SyntaxError, e:
            # FIXME: sometimes an URLError is catched instead of a standard IOError
            try:
                dummy = e.errno
            except: 
                raise IOError, e
            
            if e.errno in (2, "socket error", "url error"): # catch IOError: No such file or directory
                self._raiseError ("%s: '%s'" %(e.strerror, e.filename), nextSibling)
            else:
                raise
        
        return subTree

    ########################################
    # update lookup dictionaries used during validation
    #
    def _updateLookupTables (self, rootNode, lookupDict):
        schemaTagDict = {"element"    : "ElementDict",
                         "complexType": "TypeDict",
                         "simpleType" : "TypeDict",
                         "group"      : "GroupDict",
                         "attributeGroup": "AttrGroupDict",
                         "attribute"     : "AttributeDict",
                        }

        # retrieve all schema tags
        for localName, lookupDictName in schemaTagDict.items():
            for node in rootNode.getChildrenNS(XSD_NAMESPACE, localName):
                targetNamespace = self._getTargetNamespace(node)
                if not lookupDict[lookupDictName].has_key((targetNamespace, node.getAttribute("name"))):
                    lookupDict[lookupDictName][(targetNamespace, node.getAttribute("name"))] = node
        
        # retrieve all identity constraints
        for identConstrTagName in ("unique", "key", "keyref"):
            identConstrNodeList = rootNode.getElementsByTagNameNS (XSD_NAMESPACE, identConstrTagName)
            for identConstrNode in identConstrNodeList:
                targetNamespace = self._getTargetNamespace(identConstrNode)
                identConstrNsLocalName = NsNameTupleFactory ( (targetNamespace, identConstrNode.getAttribute("name")) )
                if not lookupDict["IdentityConstrDict"].has_key(identConstrNsLocalName):
                    lookupDict["IdentityConstrDict"][identConstrNsLocalName] = {"Node": identConstrNode, "ValueDict":{}}

#                else:
#                    self._addError ("Duplicate identity constraint name %s found!"
#                                    %(repr(identConstrNsLocalName)), identConstrNode)

    ########################################
    # validate inputNode against complexType node
    #
    def _initInternalAttributes (self, rootNode):
        # set schema root node for all descendant nodes
        if not rootNode.getSchemaRootNode():
            for node in rootNode.getIterator():
                node.setSchemaRootNode(rootNode)


    ########################################
    # validate inputNode against complexType node
    #
    def _checkComplexTypeTag (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType=None):
        baseTypeAttributes = {}

        complexContentNode = xsdNode.getFirstChildNS(self.xsdNsURI, "complexContent")
        simpleContentNode = xsdNode.getFirstChildNS(self.xsdNsURI, "simpleContent")
        if complexContentNode != None:
            inputChildIndex, baseTypeAttributes = self._checkComplexContentTag (xsdParentNode, complexContentNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        elif simpleContentNode != None:
            inputChildIndex, baseTypeAttributes = self._checkSimpleContentTag (xsdParentNode, simpleContentNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        else:
            inputChildIndex, baseTypeAttributes = self._checkComplexTypeContent (xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
            if usedAsBaseType == None:
                self._checkMixed (xsdParentNode, xsdNode, inputNode)
        return inputChildIndex, baseTypeAttributes

    def _checkComplexContentTag (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        extensionNode = xsdNode.getFirstChildNS(self.xsdNsURI, "extension")
        if extensionNode != None:
            inputChildIndex, baseTypeAttributes = self._checkExtensionComplexContent (xsdParentNode, extensionNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        else:
            restrictionNode = xsdNode.getFirstChildNS(self.xsdNsURI, "restriction")
            if restrictionNode != None:
                inputChildIndex, baseTypeAttributes = self._checkRestrictionComplexContent (xsdParentNode, restrictionNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
            else:
                raise AttributeError, "RestrictionNode not found!"

#        if usedAsBaseType == None:
#            self._checkMixed (xsdNode, inputNode)
        return inputChildIndex, baseTypeAttributes

    def _checkSimpleContentTag (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        if inputNode.getAttribute( (XSI_NAMESPACE, "nil") ) == "true":
            if inputNode.getChildren() != [] or collapseString(inputNode.getElementValue()) != "":
                self._addError ("Element must be empty (xsi:nil='true')(1)!" , inputNode, 0)

        extensionNode = xsdNode.getFirstChildNS(self.xsdNsURI, "extension")
        if extensionNode != None:
            inputChildIndex, baseTypeAttributes = self._checkExtensionSimpleContent (xsdParentNode, extensionNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        else:
            restrictionNode = xsdNode.getFirstChildNS(self.xsdNsURI, "restriction")
            if restrictionNode != None:
                inputChildIndex, baseTypeAttributes = self._checkRestrictionSimpleContent (xsdParentNode, xsdNode, restrictionNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        return inputChildIndex, baseTypeAttributes

    def _checkExtensionComplexContent (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        baseNsName = xsdNode.getQNameAttribute("base")
        if usedAsBaseType == None: 
            extUsedAsBaseType = "extension"
        else:
            extUsedAsBaseType = usedAsBaseType
        inputChildIndex, baseTypeAttributes = self._checkComplexTypeTag (xsdParentNode, self.xsdTypeDict[baseNsName], inputNode, inputChildIndex, extUsedAsBaseType)

        inputChildIndex, baseTypeAttributes = self._checkComplexTypeContent (xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        return inputChildIndex, baseTypeAttributes

    def _checkExtensionSimpleContent (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        self._checkSimpleType (xsdNode, "base", inputNode, inputNode.getTagName(), inputNode.getElementValue(), None, checkAttribute=0)
        if xsdNode.hasAttribute("BaseTypes"):
            xsdParentNode["BaseTypes"] = xsdNode["BaseTypes"]
        inputChildIndex, baseTypeAttributes = self._checkSimpleTypeContent (xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        return inputChildIndex, baseTypeAttributes

    def _checkRestrictionComplexContent (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        # first check against base type (retrieve only the base type attributes)
        baseNsName = xsdNode.getQNameAttribute("base")
        inputChildIndex, baseTypeAttributes = self._checkComplexTypeTag (xsdParentNode, self.xsdTypeDict[baseNsName], inputNode, inputChildIndex, "restriction")

        # then check input against derived complex type
        inputChildIndex, baseTypeAttributes = self._checkComplexTypeContent (xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        return inputChildIndex, baseTypeAttributes

    def _checkRestrictionSimpleContent (self, xsdParentNode, simpleContentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        try:
            simpleTypeReturnDict = {"BaseTypes":[], "primitiveType":None}
            self.simpleTypeVal.checkSimpleTypeDef (inputNode, simpleContentNode, inputNode.getTagName(), inputNode.getElementValue(), simpleTypeReturnDict, idCheck=1)
            xsdNode["BaseTypes"] = string.join (simpleTypeReturnDict["BaseTypes"], " ")
        except SimpleTypeError, errInst:
            self._addError (errInst.args[0], inputNode)

        inputChildIndex, baseTypeAttributes = self._checkSimpleTypeContent (xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes)
        return inputChildIndex, baseTypeAttributes

    def _checkComplexTypeContent (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        if inputNode.getAttribute((XSI_NAMESPACE, "nil")) == "true":
            if inputNode.getChildren() != [] or collapseString(inputNode.getElementValue()) != "":
                self._addError ("Element must be empty (xsi:nil='true')(2)!" , inputNode, 0)
        else:
            childTags = inputNode.getChildren()
            if usedAsBaseType in (None, "extension"):
                validChildTags = xsdNode.getChildren()
                for validChildTag in validChildTags:
                    if validChildTag.getLocalName() not in ("attribute", "attributeGroup", "anyAttribute"):
                        inputChildIndex = self._checkParticle (validChildTag, inputNode, childTags, inputChildIndex)

                if usedAsBaseType == None and inputChildIndex < len (childTags):
                    inputNsName = inputNode.getNsName()
                    childNsName = childTags[inputChildIndex].getNsName()
                    self._addError ("Unexpected or invalid child tag %s found in tag %s!"
                                    %(repr(childNsName), repr(inputNsName)), childTags[inputChildIndex])

        if usedAsBaseType in (None,):
            self._checkAttributeTags (xsdParentNode, xsdNode, inputNode, baseTypeAttributes)

        if usedAsBaseType in ("restriction", "extension"):
            self._updateAttributeDict (xsdNode, baseTypeAttributes)

        return inputChildIndex, baseTypeAttributes

    def _checkSimpleTypeContent (self, xsdParentNode, xsdNode, inputNode, inputChildIndex, usedAsBaseType, baseTypeAttributes):
        if inputNode.getChildren() != []:
            raise TagException ("No child tags are allowed for element %s!" %repr(inputNode.getNsName()), inputNode)

        if usedAsBaseType in (None,):
            self._checkAttributeTags (xsdParentNode, xsdNode, inputNode, baseTypeAttributes)

        if usedAsBaseType in ("restriction", "extension"):
            self._updateAttributeDict (xsdNode, baseTypeAttributes)

        return inputChildIndex, baseTypeAttributes


    ########################################
    # validate mixed content (1)
    #
    def _checkMixed (self, xsdParentNode, xsdNode, inputNode):
        if xsdNode.getAttributeOrDefault ("mixed", "false") == "false":
            if not collapseString(inputNode.getElementValue()) in ("", " "):
                self._addError ("Mixed content not allowed for %s!" %repr(inputNode.getTagName()), inputNode)
        else: # mixed = true
            self._checkUrType(xsdParentNode, inputNode)


    ########################################
    # check ur-type
    #
    def _checkUrType (self, xsdNode, inputNode):
        prefix = xsdNode.getPrefix()
        if prefix:
            xsdNode["__CONTENTTYPE__"] = "%s:string" %xsdNode.getPrefix()
        else:
            xsdNode["__CONTENTTYPE__"] = "string"
        self._checkElementValue (xsdNode, "__CONTENTTYPE__", inputNode)
        xsdNode.removeAttribute("__CONTENTTYPE__")
    

    ########################################
    # validate inputNodeList against xsdNode
    #
    def _checkList (self, elementMethod, xsdNode, inputParentNode, inputNodeList, currIndex):
        minOccurs = string.atoi(xsdNode.getAttributeOrDefault("minOccurs", "1"))
        maxOccurs = xsdNode.getAttributeOrDefault("maxOccurs", "1")
        if maxOccurs != "unbounded":
            maxOccurs = string.atoi(maxOccurs)
        else:
            maxOccurs = -1
        occurs = 0
        while maxOccurs == -1 or occurs < maxOccurs:
            try:
                newIndex = elementMethod (xsdNode, inputParentNode, inputNodeList, currIndex)
                occurs += 1
                if newIndex > currIndex:
                    currIndex = newIndex
                else:
                    break # no suitable element found
            except TagException, errInst:
                break

        if occurs == 0 and minOccurs > 0:
            raise errInst
        elif occurs < minOccurs:
            expInputTagName = xsdNode.getAttribute("name")
            if expInputTagName == None:
                expInputTagName = xsdNode.getQNameAttribute("ref")
            raise TagException ("minOccurs (%d) of child tag %s in tag %s not available (only %d)!"
                                %(minOccurs, repr(expInputTagName), repr(inputParentNode.getTagName()), occurs), inputParentNode, 1)

        return currIndex

    ########################################
    # validate inputNode against element node
    #
    def _checkElementTag (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        if xsdNode.hasAttribute("ref"):
            refNsName = xsdNode.getQNameAttribute("ref")
            currIndex = self._checkElementTag (self.xsdElementDict[refNsName], inputParentNode, inputNodeList, currIndex)
        else:
            nameAttr = xsdNode.getAttribute ("name")

            if currIndex >= len (inputNodeList):
                raise TagException ("Missing child tag %s in tag %s!" %(repr(nameAttr), repr(inputParentNode.getTagName())), inputParentNode, 1)

            inputNode = inputNodeList[currIndex]
            if nameAttr != inputNode.getLocalName():
                raise TagException ("Missing child tag %s in tag %s!" %(repr(nameAttr), repr(inputParentNode.getTagName())), inputNode, 0)

            # store reference to XSD definition node
            inputNode.setXsdNode(xsdNode)

            currIndex = currIndex + 1

            self._checkInputElementForm (xsdNode, nameAttr, inputNode)

            if (xsdNode.getFirstChild() == None and 
                not xsdNode.hasAttribute("type") and 
                not inputNode.hasAttribute((XSI_NAMESPACE, "type")) ):
                self._checkUrType(xsdNode, inputNode)
                # ur-type => try to check children of input node
                for inputChild in inputNode.getChildren():
                    try:
                        if self.xsdElementDict.has_key(inputChild.getNsName()):
                            self._checkElementTag (self.xsdElementDict[inputChild.getNsName()], inputNode, (inputChild,), 0)
                    except TagException, errInst:
                        self._addError (errInst.errstr, errInst.node, errInst.endTag)
                return currIndex
            
            complexTypeNode = xsdNode.getFirstChildNS (self.xsdNsURI, "complexType")
            if not inputNode.hasAttribute((XSI_NAMESPACE, "type")):
                typeNsName = xsdNode.getQNameAttribute ("type")
            else:
                # overloaded type is used
                typeNsName = inputNode.getQNameAttribute((XSI_NAMESPACE, "type"))
                if not self.xsdTypeDict.has_key(typeNsName):
                    self._addError ("Unknown overloaded type %s!" %(repr(typeNsName)), inputNode, 0)
                    return currIndex

            if self.xsdTypeDict.has_key (typeNsName):
                typeType = self.xsdTypeDict[typeNsName].getLocalName()
                if typeType == "complexType":
                    complexTypeNode = self.xsdTypeDict[typeNsName]
                # else simpleType => pass, handled later on

            if complexTypeNode != None:
                try:
                    self._checkComplexTypeTag (xsdNode, complexTypeNode, inputNode, 0)
                except TagException, errInst:
                    self._addError (errInst.errstr, errInst.node, errInst.endTag)
                    return currIndex
            else:
                self._checkElementSimpleType (xsdNode, "type", inputNode)

            # check unique attributes and keys
            childUniqueDefList = xsdNode.getChildrenNS (self.xsdNsURI, "unique")
            for childUniqueDef in childUniqueDefList:
                self._checkIdentityConstraint (childUniqueDef, inputNode)

            childKeyDefList = xsdNode.getChildrenNS (self.xsdNsURI, "key")
            for childKeyDef in childKeyDefList:
                self._checkIdentityConstraint (childKeyDef, inputNode)

            childKeyrefDefList = xsdNode.getChildrenNS (self.xsdNsURI, "keyref")
            for childKeyrefDef in childKeyrefDefList:
                self.checkKeyrefList.append ((inputNode, childKeyrefDef))

        return currIndex


    ########################################
    # validate element inputNode against simple type definition
    #
    def _checkElementSimpleType (self, xsdNode, xsdTypeAttr, inputNode):
        if inputNode.getChildren() != []:
            raise TagException ("No child tags are allowed for element %s!" %(repr(inputNode.getNsName())), inputNode)

        self._checkElementValue (xsdNode, xsdTypeAttr, inputNode)

        self._checkAttributeTags (xsdNode, xsdNode, inputNode, {})


    ########################################
    # validate inputNode against simple type definition
    #
    def _checkElementValue (self, xsdNode, xsdTypeAttr, inputNode):
        fixedValue = xsdNode.getAttribute("fixed")
        if inputNode.getAttribute((XSI_NAMESPACE, "nil")) == "true" and fixedValue != None:
            self._addError ("There must be no fixed value for Element because xsi:nil='true' is specified!" , inputNode)

        if inputNode.getAttribute((XSI_NAMESPACE, "nil")) != "true" and inputNode.getElementValue() == "":
            if xsdNode.hasAttribute("default"):
                inputNode.setElementValue(xsdNode["default"])
            if fixedValue != None:
                inputNode.setElementValue(fixedValue)

        self._checkSimpleType (xsdNode, xsdTypeAttr, inputNode, inputNode.getTagName(), inputNode.getElementValue(), fixedValue, checkAttribute=0)


    ########################################
    # validate inputNode against simple type definition
    #
    def _checkSimpleType (self, xsdNode, xsdTypeAttr, inputNode, attrName, attrValue, fixedValue, checkAttribute=0, checkId=1):
        retValue = None

        if checkAttribute == 0 and inputNode.hasAttribute((XSI_NAMESPACE, "nil")):
            if (inputNode[(XSI_NAMESPACE, "nil")] == "true" and
                collapseString(inputNode.getElementValue()) != ""):
                self._addError ("Element must be empty (xsi:nil='true')(3)!" , inputNode, 0)
            return retValue
        
        try:
            simpleTypeReturnDict = {"BaseTypes":[], "primitiveType":None}
            fixedValueReturnDict = {"BaseTypes":[], "primitiveType":None}
            simpleTypeNode  = xsdNode.getFirstChildNS (self.xsdNsURI, "simpleType")
            if simpleTypeNode != None:
                self.simpleTypeVal.checkSimpleTypeDef (inputNode, simpleTypeNode, attrName, attrValue, simpleTypeReturnDict, checkId)
                if fixedValue != None:
                    self.simpleTypeVal.checkSimpleTypeDef (inputNode, simpleTypeNode, attrName, fixedValue, fixedValueReturnDict, idCheck=0)
            elif (xsdNode.getFirstChildNS (self.xsdNsURI, "complexType") != None and
                  xsdNode.getFirstChildNS (self.xsdNsURI, "complexType").getAttribute("mixed") == "false"):
                self._addError ("Attribute %s requires a simple or mixed type!" %repr(attrName), inputNode)
            else:
                typeNsName = xsdNode.getQNameAttribute (xsdTypeAttr)
                if typeNsName != (None, None):
                    self.simpleTypeVal.checkSimpleType (inputNode, attrName, typeNsName, attrValue, simpleTypeReturnDict, checkId)
                # TODO: What to check if no type is specified for the element?
                    if fixedValue != None:
                        self.simpleTypeVal.checkSimpleType (inputNode, attrName, typeNsName, fixedValue, fixedValueReturnDict, idCheck=0)
            
            xsdNode["BaseTypes"] = string.join (simpleTypeReturnDict["BaseTypes"], " ")
            xsdNode["primitiveType"] = str(simpleTypeReturnDict["primitiveType"])

            retValue = simpleTypeReturnDict
            if simpleTypeReturnDict.has_key("wsAction"):
                if checkAttribute:
                    attrValue = inputNode.processWsAttribute(attrName, simpleTypeReturnDict["wsAction"])
                else:
                    attrValue = inputNode.processWsElementValue(simpleTypeReturnDict["wsAction"])

            if fixedValue != None:
                if fixedValueReturnDict.has_key("orderedValue"):
                    fixedValue = fixedValueReturnDict["orderedValue"]
                elif fixedValueReturnDict.has_key("adaptedAttrValue"):
                    fixedValue = fixedValueReturnDict["adaptedAttrValue"]
                if simpleTypeReturnDict.has_key("orderedValue"):
                    attrValue = simpleTypeReturnDict["orderedValue"]
                if attrValue != fixedValue:
                    if checkAttribute:
                        self._addError ("Attribute %s must have fixed value %s!" %(repr(attrName), repr(fixedValue)), inputNode)
                    else:
                        self._addError ("Element must have fixed value %s!" %repr(fixedValue), inputNode)
                        
        except SimpleTypeError, errInst:
            self._addError (errInst.args[0], inputNode)

        return retValue

    ########################################
    # validate inputNode against sequence node
    #
    def _checkSequenceTag (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        for xsdChildNode in xsdNode.getChildren():
            currIndex = self._checkParticle (xsdChildNode, inputParentNode, inputNodeList, currIndex)
        return currIndex


    ########################################
    # validate inputNode against choice node
    #
    def _checkChoiceTag (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        childFound = 0
        exceptionRaised = 0
        for xsdChildNode in xsdNode.getChildren():
            try:
                newIndex = self._checkParticle (xsdChildNode, inputParentNode, inputNodeList, currIndex)
                if newIndex > currIndex:
                    currIndex = newIndex
                    childFound = 1
                    break
                else:
                    exceptionRaised = 0
            except TagException, errInst:
                exceptionRaised = 1
        else:
            if not childFound and exceptionRaised:
                if currIndex < len(inputNodeList):
                    currNode = inputNodeList[currIndex]
                    endTag = 0
                else:
                    currNode = inputParentNode
                    endTag = 1
                raise TagException ("No suitable child tag for choice found!", currNode, endTag)

        return currIndex


    ########################################
    # validate inputNode against group node
    #
    def _checkGroupTag (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        if xsdNode.hasAttribute("ref"):
            refNsName = xsdNode.getQNameAttribute("ref")
            currIndex = self._checkGroupTag (self.xsdGroupDict[refNsName], inputParentNode, inputNodeList, currIndex)
        else:
            for xsdChildNode in xsdNode.getChildren():
                currIndex = self._checkParticle (xsdChildNode, inputParentNode, inputNodeList, currIndex)
        return currIndex


    ########################################
    # validate inputNode against all node
    #
    def _checkAllTag (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        oldIndex = currIndex
        xsdChildDict = {}
        for xsdChildNode in xsdNode.getChildren():
            if xsdChildNode.getNsName() != (XSD_NAMESPACE, "annotation"):
                xsdChildDict[xsdChildNode] = 0
        while (currIndex < len(inputNodeList)) and (0 in xsdChildDict.values()):
            currNode = inputNodeList[currIndex]
            for xsdChildNode in xsdChildDict.keys():
                try:
                    newIndex = self._checkParticle (xsdChildNode, inputParentNode, inputNodeList, currIndex)
                    if newIndex == currIndex:
                        continue
                except TagException, errInst:
                    continue

                if xsdChildDict[xsdChildNode] == 0:
                    xsdChildDict[xsdChildNode] = 1
                    currIndex = newIndex
                    break
                else:
                    raise TagException ("Ambiguous child tag %s found in all-group!" %repr(currNode.getTagName()), currNode)
            else:
                raise TagException ("Unexpected child tag %s for all-group found!" %repr(currNode.getTagName()), currNode)

        for xsdChildNode, occurs in xsdChildDict.items():
            if xsdChildNode.getAttributeOrDefault("minOccurs", "1") != "0" and occurs == 0:
                raise TagException ("Child tag %s missing in all-group (%s)" %(repr(xsdChildNode.getAttribute("name")), repr(inputParentNode.getTagName())), inputParentNode)

        return currIndex


    ########################################
    # validate inputNode against any node
    #
    def _checkAnyTag (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        if currIndex >= len (inputNodeList):
            raise TagException ("Missing child tag (anyTag) in tag %s!" %repr(inputParentNode.getTagName()), inputParentNode, 1)

        inputNode = inputNodeList[currIndex]
        inputNamespace = inputNode.getNamespaceURI()
        self._checkWildcardElement (xsdNode, inputNode, inputNamespace)

        currIndex = currIndex + 1
        return currIndex


    ########################################
    # validate inputNode against particle
    #
    def _checkParticle (self, xsdNode, inputParentNode, inputNodeList, currIndex):
        xsdTagName = xsdNode.getLocalName()
        if xsdTagName == "element":
            currIndex = self._checkList (self._checkElementTag, xsdNode, inputParentNode, inputNodeList, currIndex)
        elif xsdTagName == "choice":
            currIndex = self._checkList (self._checkChoiceTag, xsdNode, inputParentNode, inputNodeList, currIndex)
        elif xsdTagName == "sequence":
            currIndex = self._checkList (self._checkSequenceTag, xsdNode, inputParentNode, inputNodeList, currIndex)
        elif xsdTagName == "group":
            currIndex = self._checkList (self._checkGroupTag, xsdNode, inputParentNode, inputNodeList, currIndex)
        elif xsdTagName == "all":
            currIndex = self._checkList (self._checkAllTag, xsdNode, inputParentNode, inputNodeList, currIndex)
        elif xsdTagName == "any":
            currIndex = self._checkList (self._checkAnyTag, xsdNode, inputParentNode, inputNodeList, currIndex)
        elif xsdTagName == "annotation":
            # TODO: really nothing to check??
            pass
        else:
            self._addError ("Internal error: Invalid tag %s found!" %repr(xsdTagName))
        return currIndex


    ########################################
    # validate attributes of inputNode against complexType node
    #
    def _checkAttributeTags (self, parentNode, xsdNode, inputNode, validAttrDict):
        # retrieve all valid attributes for this element from the schema file
        self._updateAttributeDict (xsdNode, validAttrDict)
        inputAttrDict = {}
        for iAttrName, iAttrValue in inputNode.getAttributeDict().items():
            # skip namespace declarations
            if iAttrName[0] != XMLNS_NAMESPACE and iAttrName[1] != "xmlns":
                inputAttrDict[iAttrName] = iAttrValue

        for qAttrName, validAttrEntry in validAttrDict.items():
            attrRefNode = validAttrEntry["RefNode"]
            # global attributes use always form "qualified"
            if self.xsdAttributeDict.has_key(qAttrName) and self.xsdAttributeDict[qAttrName] == attrRefNode:
                attributeForm = "qualified"
            else:
                attributeForm = attrRefNode.getAttributeOrDefault ("form", self._getAttributeFormDefault(xsdNode))
            attrRefNode.setAttribute ("form", attributeForm)
            self._checkAttributeTag (qAttrName, validAttrEntry["Node"], attrRefNode, inputNode, inputAttrDict)

        for inputAttribute in inputAttrDict.keys():
            if inputAttribute == (XSI_NAMESPACE, "type"):
                pass # for attribute xsi:type refer _checkElementTag
            elif inputAttribute == (XSI_NAMESPACE, "nil"):
                if parentNode.getAttributeOrDefault ("nillable", "false") == "false":
                    self._addError ("Tag %s hasn't been defined as nillable!" %repr(inputNode.getTagName()), inputNode)
            elif inputNode == self.inputRoot and inputAttribute in ((XSI_NAMESPACE, "noNamespaceSchemaLocation"), (XSI_NAMESPACE, "schemaLocation")):
                pass
            elif validAttrDict.has_key("__ANY_ATTRIBUTE__"):
                xsdNode = validAttrDict["__ANY_ATTRIBUTE__"]["Node"]
                try:
                    inputNamespace = inputAttribute[0]
                    if inputAttribute[0] == None and xsdNode.getAttribute("form") == "unqualified":
                        # TODO: Check: If only local namespace is allowed, do not use target namespace???
                        if xsdNode.getAttribute("namespace") != "##local":
                            inputNamespace = self._getTargetNamespace(xsdNode)
                    self._checkWildcardAttribute (xsdNode, inputNode, inputAttribute, inputNamespace, inputAttrDict)
                except TagException:
                    self._addError ("Unexpected attribute %s in Tag %s!" %(repr(inputAttribute), repr(inputNode.getTagName())), inputNode)
            else:
                self._addError ("Unexpected attribute %s in Tag %s!" %(repr(inputAttribute), repr(inputNode.getTagName())), inputNode)


    ########################################
    # validate one attribute (defined by xsdNode) of inputNode
    #
    def _checkAttributeTag (self, qAttrName, xsdAttrNode, xsdAttrRefNode, inputNode, inputAttrDict):
        targetNamespace = self._getTargetNamespace(xsdAttrNode)
        if qAttrName[0] == targetNamespace and xsdAttrRefNode.getAttribute("form") == "unqualified":
            qAttrName = NsNameTupleFactory( (None, qAttrName[1]) )
        
        use = xsdAttrNode.getAttribute("use")
        if use == None: use = xsdAttrRefNode.getAttributeOrDefault ("use", "optional")
        fixedValue = xsdAttrNode.getAttribute("fixed")
        if fixedValue == None: 
            fixedValue = xsdAttrRefNode.getAttribute("fixed")

        if inputAttrDict.has_key(qAttrName):
            if use == "prohibited":
                self._addError ("Attribute %s is prohibited in this context!" %repr(qAttrName[1]), inputNode)
        elif inputAttrDict.has_key((targetNamespace, qAttrName[1])):
            self._addError ("Local attribute %s must be unqualified!" %(repr(qAttrName)), inputNode)
            del inputAttrDict[(targetNamespace, qAttrName[1])]
        elif inputAttrDict.has_key((None, qAttrName[1])) and qAttrName[0] == targetNamespace:
            self._addError ("Attribute %s must be qualified!" %repr(qAttrName[1]), inputNode)
            del inputAttrDict[(None, qAttrName[1])]
        else:
            if use == "required":
                self._addError ("Attribute %s is missing!" %(repr(qAttrName)), inputNode)
            elif use == "optional":
                if xsdAttrRefNode.hasAttribute("default"):
                    if not (inputNode.getNsName() == (XSD_NAMESPACE, "element") and
                            inputNode.hasAttribute("ref") and
                            xsdAttrRefNode.getAttribute("name") == "nillable"):
                        defaultValue = xsdAttrRefNode.getAttribute("default")
                        inputNode.setAttribute(qAttrName, defaultValue)
                        inputAttrDict[qAttrName] = defaultValue
                elif fixedValue != None:
                    inputNode.setAttribute(qAttrName, fixedValue)
                    inputAttrDict[qAttrName] = fixedValue

        if inputAttrDict.has_key(qAttrName):
            attributeValue = inputAttrDict[qAttrName]
            self._checkSimpleType (xsdAttrRefNode, "type", inputNode, qAttrName, attributeValue, fixedValue, 1)
            del inputAttrDict[qAttrName]
            inputNode.setXsdAttrNode(qAttrName, xsdAttrRefNode)


    ########################################
    # update dictionary of valid attributes
    #
    def _updateAttributeDict (self, xsdNode, validAttrDict, checkForDuplicateAttr=0, recursionKeys=None):
        # TODO: Why can recursionKeys not be initialized by default variable??
        if recursionKeys == None: recursionKeys = {} 
        validAttributeNodes = xsdNode.getChildrenNS(self.xsdNsURI, "attribute")
        for validAttrGroup in xsdNode.getChildrenNS(self.xsdNsURI, "attributeGroup"):
            refNsName = validAttrGroup.getQNameAttribute("ref")
            if self.xsdAttrGroupDict.has_key(refNsName):
                if recursionKeys.has_key(refNsName):
                    self._addError ("Circular definition for attribute group %s detected!" %(repr(refNsName)), validAttrGroup)
                    continue
                recursionKeys[refNsName] = 1
                self._updateAttributeDict(self.xsdAttrGroupDict[refNsName], validAttrDict, checkForDuplicateAttr, recursionKeys)
               

        for validAttributeNode in validAttributeNodes:
            if validAttributeNode.hasAttribute("ref"):
                attrKey = validAttributeNode.getQNameAttribute("ref")
                attributeRefNode = self.xsdAttributeDict[attrKey]
            else:
                attrKey = validAttributeNode.getQNameAttribute("name")
                attrKey = (self._getTargetNamespace(validAttributeNode), validAttributeNode.getAttribute("name"))
                attributeRefNode = validAttributeNode
                
            if checkForDuplicateAttr and validAttrDict.has_key(attrKey):
                self._addError ("Duplicate attribute %s found!" %repr(attrKey), validAttributeNode)
            else:
                validAttrDict[attrKey] = {"Node":validAttributeNode, "RefNode":attributeRefNode}

        anyAttributeNode = xsdNode.getFirstChildNS(self.xsdNsURI, "anyAttribute")
        if anyAttributeNode != None:
            validAttrDict["__ANY_ATTRIBUTE__"] = {"Node":anyAttributeNode, "RefNode":anyAttributeNode}


    ########################################
    # validate wildcard specification of anyElement
    #
    def _checkWildcardElement (self, xsdNode, inputNode, inputNamespace):
        processContents = xsdNode.getAttributeOrDefault("processContents", "lax")

        self._checkInputNamespace (xsdNode, inputNode, inputNamespace)

        inputNsName = inputNode.getNsName()
        if processContents == "skip":
            pass
        else:
            if inputNode.hasAttribute((XSI_NAMESPACE, "type")):
                # overloaded type is used
                typeNsName = inputNode.getQNameAttribute((XSI_NAMESPACE, "type"))
                if not self.xsdTypeDict.has_key(typeNsName):
                    self._addError ("Unknown overloaded type %s!" %(repr(typeNsName)), inputNode, 0)
                else:
                    typeType = self.xsdTypeDict[typeNsName].getLocalName()
                    if typeType == "complexType":
                        try:
                            self._checkComplexTypeTag (None, self.xsdTypeDict[typeNsName], inputNode, 0)
                        except TagException, errInst:
                            self._addError (errInst.errstr, errInst.node, errInst.endTag)
                    else:
                        simpleTypeReturnDict = {"BaseTypes":[], "primitiveType":None}
                        try:
                            self.simpleTypeVal.checkSimpleType (inputNode, inputNode.getLocalName(), typeNsName, inputNode.getElementValue(), simpleTypeReturnDict, idCheck=1)
                        except SimpleTypeError, errInst:
                            self._addError (errInst.args[0], inputNode)

            elif self.xsdElementDict.has_key(inputNsName):
                self._checkElementTag (self.xsdElementDict[inputNsName], None, (inputNode,), 0)

            elif processContents == "strict":
                self._addError ("Element definition %s not found in schema file!" %repr(inputNsName), inputNode)
                

    ########################################
    # validate wildcard specification of anyElement/anyAttribute
    #
    def _checkWildcardAttribute (self, xsdNode, inputNode, qAttrName, inputNamespace, inputAttrDict):
        processContents = xsdNode.getAttributeOrDefault("processContents", "strict")

        self._checkInputNamespace (xsdNode, inputNode, inputNamespace)

        if processContents == "skip":
            pass
        elif processContents == "lax":
            if self.xsdAttributeDict.has_key(qAttrName):
                attrNode = self.xsdAttributeDict[qAttrName]
                self._checkAttributeTag (qAttrName, attrNode, attrNode, inputNode, inputAttrDict)
        elif processContents == "strict":
            if self.xsdAttributeDict.has_key(qAttrName):
                attrNode = self.xsdAttributeDict[qAttrName]
                self._checkAttributeTag (qAttrName, attrNode, attrNode, inputNode, inputAttrDict)
            else:
                self._addError ("Attribute definition %s not found in schema file!" %repr(qAttrName), inputNode)
                

    ########################################
    # validate wildcard specification of anyElement/anyAttribute
    #
    def _checkInputNamespace (self, xsdNode, inputNode, inputNamespace):
        targetNamespace = self._getTargetNamespace(xsdNode)
        namespaces = xsdNode.getAttributeOrDefault("namespace", "##any")
        if namespaces == "##any":
            pass   # nothing to check
        elif namespaces == "##other":
            if inputNamespace == targetNamespace or inputNamespace == None:
                raise TagException ("Node or attribute must not be part of target namespace or local!", inputNode)
        else:
            for namespace in string.split(collapseString(namespaces), " "):
                if namespace == "##local" and inputNamespace == None:
                    break
                elif namespace == "##targetNamespace" and inputNamespace == targetNamespace:
                    break
                elif namespace == inputNamespace:
                    break
            else:
                raise TagException ("Node or attribute is not part of namespace %s!" %repr(namespaces), inputNode)


    ########################################
    # validate unique and key definition
    #
    def _checkIdentityConstraint (self, identityConstrNode, inputNode):
        identConstrTag = identityConstrNode.getLocalName()
        identConstrName = identityConstrNode.getAttribute ("name")
        identConstrNsLocalName = (self._getTargetNamespace(identityConstrNode), identConstrName)
        selectorXPathNode = identityConstrNode.getFirstChildNS (self.xsdNsURI, "selector")
        selectorNodeList, dummy, dummy = self._getXPath (inputNode, selectorXPathNode)
        
        valueDict = {}
        for selectorNode in selectorNodeList:
            fieldXPathNodeList = identityConstrNode.getChildrenNS (self.xsdNsURI, "field")
            keyValue = []
            baseTypesList = []
            for fieldXPathNode in fieldXPathNodeList:
                fieldChildList, attrNodeList, attrName = self._getXPath (selectorNode, fieldXPathNode, identConstrTag)
                if len(fieldChildList) > 1:
                    self._addError ("The field xPath %s of %s %s must evaluate to exactly 0 or 1 node!" %(repr(fieldXPathNode["xpath"]), repr(identConstrTag), repr(identConstrName)), selectorNode)
                    return

                for fieldChild in fieldChildList:
                    if attrNodeList == []:
                        inputChild = fieldChild
                        try:
                            baseTypes = self._setBaseTypes(fieldChild.getXsdNode())
                        except:
                            baseTypes = ((XSD_NAMESPACE, "anyType"),)
                        value = fieldChild.getElementValue()
                    else:
                        inputChild = attrNodeList[0]
                        try:
                            baseTypes = self._setBaseTypes(attrNodeList[0].getXsdAttrNode(attrName))
                        except:
                            baseTypes = ((XSD_NAMESPACE, "anyType"),)
                        value = fieldChild
                    if baseTypes != None:
                        if baseTypes[0] in ((XSD_NAMESPACE, "anyType"), (XSD_NAMESPACE, "anySimpleType")):
                            overloadedType = inputChild.getQNameAttribute((XSI_NAMESPACE, "type"))
                            if overloadedType != (None, None):
                                baseTypes = [inputChild.getQNameAttribute((XSI_NAMESPACE, "type")),]
                    else:
                        self._addError ("Identity constraint does not have a simple type!", inputChild)
                        continue
                        
                    baseTypesList.append(baseTypes)
                    for baseType in baseTypes:
                        try:
                            value = self._getOrderedValue (inputChild, attrName, baseType, value)
                            break
                        except SimpleTypeError, errInst:
                            pass
                    keyValue.append (value)

            if keyValue != []:
                keyValue = tuple(keyValue)
                if not valueDict.has_key (keyValue):
                    valueDict[keyValue] = 1
                    self.xsdIdentityConstrDict[identConstrNsLocalName]["ValueDict"][keyValue] = baseTypesList
                else:
                    if len(keyValue) == 1:
                        self._addError ("Duplicate identity constraint values %s found for identity contraint %s!" %(repr(keyValue[0]), repr(identConstrName)), selectorNode)
                    else:
                        self._addError ("Duplicate identity constraint values %s found for identity contraint %s!" %(repr(keyValue), repr(identConstrName)), selectorNode)

    ########################################
    # validate unique and key definition
    #
    def _checkKeyRefConstraint (self, keyrefNode, inputNode):
        keyRefName = keyrefNode.getAttribute ("name")
        keyReference = keyrefNode.getQNameAttribute ("refer")

        selectorXPathNode = keyrefNode.getFirstChildNS (self.xsdNsURI, "selector")
        selectorNodeList, dummy, dummy = self._getXPath (inputNode, selectorXPathNode)
        for selectorNode in selectorNodeList:
            fieldXPathNodeList = keyrefNode.getChildrenNS(self.xsdNsURI, "field")
            keyValue = []
            for fieldXPathNode in fieldXPathNodeList:
                fieldChildList, attrNodeList, attrName = self._getXPath (selectorNode, fieldXPathNode, "keyref")
                if len(fieldChildList) > 1:
                    self._addError ("The field xPath of keyref %s must evaluate to exactly 0 or 1 node!" %repr(keyRefName), fieldXPathNode)
                    return

                for fieldChild in fieldChildList:
                    if attrNodeList == []:
                        inputChild = fieldChild
                        baseTypes = self._setBaseTypes(fieldChild.getXsdNode())
                        value = fieldChild.getElementValue()
                    else:
                        inputChild = attrNodeList[0]
                        baseTypes = self._setBaseTypes(attrNodeList[0].getXsdAttrNode(attrName))
                        value = fieldChild

                    if baseTypes != None:
                        for baseType in baseTypes:
                            try:
                                value = self._getOrderedValue (inputChild, attrName, baseType, value)
                                break
                            except SimpleTypeError, errInst:
                                pass
                    keyValue.append (value)

            keyValue = tuple(keyValue)
            if keyValue != ():
                if not self.xsdIdentityConstrDict[keyReference]["ValueDict"].has_key (keyValue):
                    self._addError ("Key reference value %s is undefined for key type %s!" %(repr(keyValue), repr(keyReference)), selectorNode)
                else:
                    baseTypesList = self.xsdIdentityConstrDict[keyReference]["ValueDict"][keyValue]
                    for fieldXPathNode, baseTypes in zip(fieldXPathNodeList, baseTypesList):
                        fieldChildList, attrNodeList, attrName = self._getXPath (selectorNode, fieldXPathNode, "keyref")
                        if attrNodeList == []:
                            inputChild = fieldChildList[0]
                            refBaseTypes = self._setBaseTypes(fieldChildList[0].getXsdNode())
                        else:
                            inputChild = attrNodeList[0]
                            refBaseTypes = self._setBaseTypes(inputChild.getXsdAttrNode(attrName))
                        if baseTypes[0] not in refBaseTypes and refBaseTypes[0] not in baseTypes:
                            if baseTypes[0] != (XSD_NAMESPACE, "anyType") and refBaseTypes[0] != (XSD_NAMESPACE, "anyType"):
                                self._addError ("Key type and key reference type does not match (%s != %s)!" %(repr(baseTypes[0]), repr(refBaseTypes[0])), inputChild)
                    
    
    ########################################
    # check input element form
    #
    def _checkInputElementForm (self, xsdNode, xsdNodeNameAttr, inputNode):
        targetNamespace = self._getTargetNamespace(xsdNode)
        nsNameAttr = (targetNamespace, xsdNodeNameAttr)
        if self.xsdElementDict.has_key(nsNameAttr) and self.xsdElementDict[nsNameAttr] == xsdNode:
            elementForm = "qualified"
        else:
            elementForm = xsdNode.getAttributeOrDefault ("form", self._getElementFormDefault(xsdNode))
        if elementForm == "qualified":
            if inputNode.getNamespaceURI() == None:
                if targetNamespace != None:
                    self._addError ("Element %s must be qualified!" %repr(xsdNodeNameAttr), inputNode)
            elif inputNode.getNamespaceURI() != targetNamespace:
                self._addError ("%s undefined in specified namespace!" %repr(xsdNodeNameAttr), inputNode)
        elif elementForm == "unqualified" and inputNode.getNamespaceURI() != None:
            self._addError ("Local element %s must be unqualified!" %repr(xsdNodeNameAttr), inputNode)


    ########################################
    # retrieve ordered value and base types of given typeNsName
    #
    def _getOrderedValue (self, inputNode, attrName, typeNsName, attrValue):
        simpleTypeReturnDict = {"BaseTypes":[], "primitiveType":None}
        self.simpleTypeVal.checkSimpleType (inputNode, attrName, typeNsName, attrValue, simpleTypeReturnDict, idCheck=1)
        if simpleTypeReturnDict.has_key("orderedValue"):
            attrValue = simpleTypeReturnDict["orderedValue"]
        return attrValue


    ########################################
    # retrieve nodes/attributes specified by given xPath
    #
    def _getXPath (self, node, xPathNode, identityConstraint=None):
        xPath = xPathNode.getAttribute("xpath")
        try:
            attrIgnoreList = [(XSI_NAMESPACE, "nil")]
            childList, attrNodeList, attrName = node.getXPathList (xPath, namespaceRef=xPathNode, useDefaultNs=0, attrIgnoreList=attrIgnoreList)
        except Exception, errInst:
            self._addError (errInst.args, node)
            childList = []
            attrNodeList = []
            attrName = None

        if childList == []:
            if identityConstraint == "key":
                self.errorHandler.addError ("Key is missing! XPath = %s!" %repr(xPath), node)
            elif identityConstraint in ("unique", "keyref"):
                self.errorHandler.addWarning ("Identity constraint is missing! XPath = %s!" %repr(xPath), node)
        return childList, attrNodeList, attrName


    ########################################
    # retrieve basetypes from XML attribute (string format)
    #
    def _setBaseTypes (self, xsdNode):
        if xsdNode.getAttribute("BaseTypes") != None:
            baseTypes = string.split(xsdNode["BaseTypes"])
            baseTypeList = map (lambda basetype: NsNameTupleFactory(basetype), baseTypes)
            if baseTypeList != []:
                return baseTypeList
            else:
                return None
        else:
            return None
    
    ########################################
    # retrieve target namespace attribute for given node
    #
    def _getTargetNamespace(self, node):
        schemaRootNode = node.getSchemaRootNode()
        return schemaRootNode.getAttribute("targetNamespace")


    ########################################
    # retrieve element form default attribute for given node
    #
    def _getElementFormDefault(self, node):
        schemaRootNode = node.getSchemaRootNode()
        return schemaRootNode.getAttributeOrDefault("elementFormDefault", "unqualified")

    ########################################
    # retrieve element form default attribute for given node
    #
    def _getAttributeFormDefault(self, node):
        schemaRootNode = node.getSchemaRootNode()
        return schemaRootNode.getAttributeOrDefault("attributeFormDefault", "unqualified")


########################################
# define own exception for XML schema validation errors
#
class TagException (StandardError):
    def __init__ (self, errstr="", node=None, endTag=0):
        self.node   = node
        self.errstr = errstr
        self.endTag = endTag
        StandardError.__init__(self)

