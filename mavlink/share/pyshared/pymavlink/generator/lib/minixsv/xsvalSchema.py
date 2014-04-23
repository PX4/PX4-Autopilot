#
# minixsv, Release 0.9.0
# file: xsvalSchema.py
#
# Derived validator class (for validation of schema files)
#
# history:
# 2004-10-07 rl   created
# 2006-08-18 rl   W3C testsuite passed for supported features
# 2007-05-24 rl   Features for release 0.8 added, several bugs fixed
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
import re
import os
from decimal             import Decimal
from ..genxmlif.xmlifUtils import collapseString
from ..minixsv             import *
from xsvalBase           import XsValBase, TagException
from xsvalUtils          import substituteSpecialEscChars

_localFacetDict = {(XSD_NAMESPACE,"list"): ("length", "minLength", "maxLength", "enumeration", "pattern", "whiteSpace"),
                   (XSD_NAMESPACE,"union"): ("enumeration", "pattern", "whiteSpace"),
                   (XSD_NAMESPACE,"anySimpleType"): ("whiteSpace"),}
###########################################################
#  Derived validator class for validating one input schema file against the XML rules file

class XsValSchema (XsValBase):

    ########################################
    # overloaded validate method
    #
    def validate (self, inputTree, xsdTree):
        XsValBase.validate(self, inputTree, xsdTree)
    
        self._initInternalAttributes (self.inputRoot)
        self._updateLookupTables (self.inputRoot, self.xsdLookupDict)

        self._includeAndImport (self.inputTree, self.inputTree, self.xsdIncludeDict, self.xsdLookupDict)

        if not self.errorHandler.hasErrors():
            # IDs must be unique within a schema file
            self.xsdIdDict = {}

            self._checkSchemaSecondLevel()

        # FIXME: Wellknown schemas are not included in the input tree although the internal attribute has been set!
        #        Better solution required than this workaround!
        self.inputRoot["__WellknownSchemasImported__"] = "false"


    ########################################
    # additional checks for schema files which are not covered by "xsStructs.xsd"
    #
    def _checkSchemaSecondLevel(self):

        targetNamespace = self.inputRoot.getAttribute("targetNamespace")
        if targetNamespace == "":
            self.errorHandler.raiseError("Empty string not allowed for target namespace!", self.inputRoot)

        self._checkElementNodesSecondLevel()
        self._checkNotationNodesSecondLevel()
        self._checkAnyNodesSecondLevel()
        self._checkGroupNodesSecondLevel()
        self._checkAttrGroupNodesSecondLevel()
        self._checkAttributeNodesSecondLevel()
        self._checkAnyAttributesSecondLevel()

        if self.errorHandler.hasErrors():
            return

        self._checkComplexTypesSecondLevel()
        self._checkSimpleTypesSecondLevel()

        self._checkParticlesSecondLevel()

        self._checkIdentityConstraintsSecondLevel()
        self._checkKeysSecondLevel()
        self._checkKeyRefsSecondLevel()

    ########################################
    # additional checks for element nodes
    #
    def _checkElementNodesSecondLevel(self):
        elementNodes = self.inputRoot.getElementsByTagNameNS (self.inputNsURI, "element")
        for elementNode in elementNodes:
            if not elementNode.hasAttribute("name") and not elementNode.hasAttribute("ref"):
                self._addError ("Element must have 'name' or 'ref' attribute!", elementNode)
                continue

            if elementNode.hasAttribute("ref"):
                for attrName in ("name", "type", "form"):
                    if elementNode.hasAttribute(attrName):
                        self._addError ("Element with 'ref' attribute must not have %s attribute!" %repr(attrName), elementNode)
                        continue

            complexTypeNode = elementNode.getFirstChildNS (self.inputNsURI, "complexType")
            simpleTypeNode = elementNode.getFirstChildNS (self.inputNsURI, "simpleType")
            if elementNode.hasAttribute("ref") and (complexTypeNode != None or simpleTypeNode != None):
                self._addError ("Element with 'ref' attribute must not have type definition!", elementNode)
                continue
            if elementNode.hasAttribute("type") and (complexTypeNode != None or simpleTypeNode != None):
                self._addError ("Element with 'type' attribute must not have type definition!", elementNode)
                continue
            
            if elementNode.hasAttribute("ref"):
                for forbiddenAttr in ("block", "nillable", "default", "fixed"):
                    if elementNode.hasAttribute(forbiddenAttr):
                        self._addError ("Element with 'ref' attribute must not have %s attribute!" %repr(forbiddenAttr), elementNode)

                self._checkReference (elementNode, self.xsdElementDict)

            if elementNode.hasAttribute("type"):
                self._checkType (elementNode, "type", self.xsdTypeDict)

            self._checkNodeId(elementNode)
            self._checkOccurs (elementNode)
            self._checkFixedDefault(elementNode)


    ########################################
    # additional checks for notation nodes
    #
    def _checkNotationNodesSecondLevel(self):
        notationNodes = self.inputRoot.getElementsByTagNameNS (self.inputNsURI, "notation")
        for notationNode in notationNodes:
            if not notationNode.hasAttribute("public") and not notationNode.hasAttribute("system"):
                self._addError ("Notation must have 'public' or 'system' attribute!", notationNode)
    
    
    ########################################
    # additional checks for anyNodes
    #
    def _checkAnyNodesSecondLevel(self):
        anyNodes = self.inputRoot.getElementsByTagNameNS (self.inputNsURI, "any")
        for anyNode in anyNodes:
            self._checkOccurs (anyNode)
            # check for unique ID
            self._checkNodeId (anyNode)


    ########################################
    # additional checks for group nodes
    #
    def _checkGroupNodesSecondLevel(self):
        groupNodes = self.inputRoot.getElementsByTagNameNS (self.inputNsURI, "group")
        for groupNode in groupNodes:
            self._checkNodeId(groupNode)
            if groupNode.hasAttribute("ref"):
                self._checkReference (groupNode, self.xsdGroupDict)
                self._checkOccurs (groupNode)
        if self.errorHandler.hasErrors():
            return
#        for groupNode in groupNodes:
#            if groupNode.hasAttribute("name"):
#                self._checkGroupNodeCircularDef(groupNode, {groupNode["name"]:1})
    
    def _checkGroupNodeCircularDef(self, groupNode, groupNameDict):
        childGroupsRefNodes, dummy, dummy = groupNode.getXPathList (".//%sgroup" %(self.inputNsPrefixString))
        for childGroupRefNode in childGroupsRefNodes:
            if childGroupRefNode.hasAttribute("ref"):
                childGroupNode = self.xsdGroupDict[childGroupRefNode.getQNameAttribute("ref")]
                if not groupNameDict.has_key(childGroupNode["name"]):
                    groupNameDict[childGroupNode["name"]] = 1
                    self._checkGroupNodeCircularDef(childGroupNode, groupNameDict)
                else:
                    self._addError ("Circular definition of group %s!" %repr(childGroupNode["name"]), childGroupNode)
                

    ########################################
    # additional checks for attributeGroup nodes
    #
    def _checkAttrGroupNodesSecondLevel(self):
        attributeGroupNodes = self.inputRoot.getElementsByTagNameNS (self.inputNsURI, "attributeGroup")
        for attributeGroupNode in attributeGroupNodes:
            if attributeGroupNode.hasAttribute("ref"):
                self._checkReference (attributeGroupNode, self.xsdAttrGroupDict)

            self._checkNodeId(attributeGroupNode)

    ########################################
    # additional checks for attribute nodes
    #
    def _checkAttributeNodesSecondLevel(self):
        attributeNodes = self.inputRoot.getElementsByTagNameNS (XSD_NAMESPACE, "attribute")
        for attributeNode in attributeNodes:
            if os.path.basename(attributeNode.getFilePath()) != "XMLSchema-instance.xsd":
                # global attributes must always be "qualified"
                if (attributeNode.getParentNode() == self.inputRoot or
                    self._getAttributeFormDefault(attributeNode) == "qualified"):
                    if self._getTargetNamespace(attributeNode) == XSI_NAMESPACE:
                        self._addError ("Target namespace of an attribute must not match '%s'!" %XSI_NAMESPACE, attributeNode)
                
            if not attributeNode.hasAttribute("name") and not attributeNode.hasAttribute("ref"):
                self._addError ("Attribute must have 'name' or 'ref' attribute!", attributeNode)
                continue

            if attributeNode.getAttribute("name") == "xmlns":
                self._addError ("Attribute must not match 'xmlns'!", attributeNode)

            if attributeNode.hasAttribute("ref"):
                if attributeNode.hasAttribute("name"):
                    self._addError ("Attribute may have 'name' OR 'ref' attribute!", attributeNode)
                if attributeNode.hasAttribute("type"):
                    self._addError ("Attribute may have 'type' OR 'ref' attribute!", attributeNode)
                if attributeNode.hasAttribute("form"):
                    self._addError ("Attribute 'form' is not allowed in this context!", attributeNode)

                if attributeNode.getFirstChildNS(XSD_NAMESPACE, "simpleType") != None:
                    self._addError ("Attribute may only have 'ref' attribute OR 'simpleType' child!", attributeNode)
                
                self._checkReference (attributeNode, self.xsdAttributeDict)

            if attributeNode.hasAttribute("type"):
                if attributeNode.getFirstChildNS(XSD_NAMESPACE, "simpleType") != None:
                    self._addError ("Attribute may only have 'type' attribute OR 'simpleType' child!", attributeNode)

                self._checkType (attributeNode, "type", self.xsdTypeDict, (XSD_NAMESPACE, "simpleType"))

            use = attributeNode.getAttribute("use")
            if use in ("required", "prohibited") and attributeNode.hasAttribute("default"):
                self._addError ("Attribute 'default' is not allowed, because 'use' is '%s'!" %(use), attributeNode)

            self._checkNodeId(attributeNode, unambiguousPerFile=0)

            self._checkFixedDefault(attributeNode)


    ########################################
    # additional checks for attribute wildcards
    #
    def _checkAnyAttributesSecondLevel(self):
        anyAttributeNodes, dummy, dummy = self.inputRoot.getXPathList (".//%sanyAttribute" %(self.inputNsPrefixString))
        for anyAttributeNode in anyAttributeNodes:
            # check for unique ID
            self._checkNodeId (anyAttributeNode)


    ########################################
    # additional checks for complex types
    #
    def _checkComplexTypesSecondLevel(self):
        prefix = self.inputNsPrefixString
        contentNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)scomplexContent/%(prefix)srestriction | .//%(prefix)scomplexContent/%(prefix)sextension" % vars())
        for contentNode in contentNodes:
            self._checkType(contentNode, "base", self.xsdTypeDict, (XSD_NAMESPACE, "complexType"))

        contentNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)ssimpleContent/%(prefix)srestriction | .//%(prefix)ssimpleContent/%(prefix)sextension" % vars())
        for contentNode in contentNodes:
            baseNsName = contentNode.getQNameAttribute("base")
            if baseNsName != (XSD_NAMESPACE, "anyType"):
                typeNsName = contentNode.getParentNode().getNsName()
                self._checkBaseType(contentNode, baseNsName, self.xsdTypeDict, typeNsName)
            else:
                self._addError ("Referred type must not be 'anyType'!", contentNode)
            # check for unique ID
            self._checkNodeId (contentNode)

        complexTypeNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)scomplexType | .//%(prefix)sextension" % vars())
        for complexTypeNode in complexTypeNodes:
            validAttrDict = {}
            # check for duplicate attributes
            self._updateAttributeDict (complexTypeNode, validAttrDict, 1)
            # check for duplicate ID attributes
            idAttrNode = None
            for key, val in validAttrDict.items():
                attrType = val["RefNode"].getQNameAttribute("type")
                if attrType == (XSD_NAMESPACE, "ID"):
                    if not idAttrNode:
                        idAttrNode = val["Node"]
                    else:
                        # TODO: check also if attribute has a type which is derived from ID!
                        self._addError ("Two attribute declarations of complex type are IDs!", val["Node"])
                        
            # check for unique ID
            self._checkNodeId (complexTypeNode)

        contentNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)scomplexType/%(prefix)s*" % vars())
        for contentNode in contentNodes:
            self._checkOccurs (contentNode)

        contentNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)scomplexContent | .//%(prefix)ssimpleContent" % vars())
        for contentNode in contentNodes:
            # check for unique ID
            self._checkNodeId (contentNode)


    ########################################
    # additional checks for simple types
    #
    def _checkParticlesSecondLevel(self):
        prefix = self.inputNsPrefixString
        # check for duplicate element names
        particleNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)sall | .//%(prefix)schoice | .//%(prefix)ssequence" % vars())
        for particleNode in particleNodes:
            elementTypeDict = {}
            elementNameDict = {}
            groupNameDict = {}
            self._checkContainedElements (particleNode, particleNode.getLocalName(), elementNameDict, elementTypeDict, groupNameDict)
            self._checkOccurs (particleNode)
            # check for unique ID
            self._checkNodeId (particleNode)
                

    def _checkContainedElements (self, node, particleType, elementNameDict, elementTypeDict, groupNameDict):
        prefix = self.inputNsPrefixString
        for childNode in node.getChildren():
            childParticleType = childNode.getLocalName()
            if childParticleType in ("sequence", "choice", "all"):
                dummy = {}
                self._checkContainedElements (childNode, childParticleType, dummy, elementTypeDict, groupNameDict)
            elif childParticleType in ("group"):
                if childNode["ref"] != None:
                    childGroupNode = self.xsdGroupDict[childNode.getQNameAttribute("ref")]
                    if not groupNameDict.has_key(childGroupNode["name"]):
                        groupNameDict[childGroupNode["name"]] = 1
                        for cChildNode in childGroupNode.getChildren():
                            if cChildNode.getLocalName() != "annotation":
                                self._checkContainedElements (cChildNode, particleType, elementNameDict, elementTypeDict, groupNameDict)
                    else:
                        self._addError ("Circular definition of group %s!" %repr(childGroupNode["name"]), childNode)
                else:
                    for cChildNode in childNode.getChildren():
                        if cChildNode.getLocalName() != "annotation":
                            self._checkContainedElements (cChildNode, particleType, elementNameDict, elementTypeDict, groupNameDict)
            else:
                if childNode.getLocalName() == "any":
                    elementName = childNode.getAttribute("namespace")
                else:
                    elementName = childNode.getAttributeOrDefault("name", childNode.getAttribute("ref"))

                if childNode.hasAttribute("type"):
                    if not elementTypeDict.has_key(elementName):
                        elementTypeDict[elementName] = childNode["type"]
                    elif childNode["type"] != elementTypeDict[elementName]:
                        self._addError ("Element %s has identical name and different types within %s!" %(repr(elementName), repr(particleType)), childNode)
                if particleType != "sequence":
                    if not elementNameDict.has_key(elementName):
                        elementNameDict[elementName] = 1
                    else:
                        self._addError ("Element %s is not unique within %s!" %(repr(elementName), repr(particleType)), childNode)


    ########################################
    # additional checks for simple types
    #
    def _checkSimpleTypesSecondLevel(self):
        prefix = self.inputNsPrefixString

        simpleTypeNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)ssimpleType" % vars())
        for simpleTypeNode in simpleTypeNodes:
            # check for unique ID
            self._checkNodeId (simpleTypeNode)
        
        restrictionNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)ssimpleType/%(prefix)srestriction" % vars())
        for restrictionNode in restrictionNodes:

            # check for unique ID
            self._checkNodeId (restrictionNode)

            if not restrictionNode.hasAttribute("base") and restrictionNode.getFirstChildNS (self.inputNsURI, "simpleType") == None:
                self._addError ("Simple type restriction must have 'base' attribute or 'simpleType' child tag!", restrictionNode)

            if restrictionNode.hasAttribute("base") and restrictionNode.getFirstChildNS (self.inputNsURI, "simpleType") != None:
                self._addError ("Simple type restriction must not have 'base' attribute and 'simpleType' child tag!", restrictionNode)

            if restrictionNode.hasAttribute("base"):
                self._checkType(restrictionNode, "base", self.xsdTypeDict)

            minExcl = restrictionNode.getFirstChildNS(self.inputNsURI, "minExclusive")
            minIncl = restrictionNode.getFirstChildNS(self.inputNsURI, "minInclusive")
            if minExcl != None and minIncl != None:
                self._addError ("Restriction attributes 'minExclusive' and 'minInclusive' cannot be defined together!", restrictionNode)
            maxExcl = restrictionNode.getFirstChildNS(self.inputNsURI, "maxExclusive")
            maxIncl = restrictionNode.getFirstChildNS(self.inputNsURI, "maxInclusive")
            if maxExcl != None and maxIncl != None:
                self._addError ("Restriction attributes 'maxExclusive' and 'maxInclusive' cannot be defined together!", restrictionNode)

        # check facets of associated primitive type
        for restrictionNode in restrictionNodes:
            try:
                if restrictionNode.hasAttribute("base"):
                    facetNsName = self._getFacetType (restrictionNode, [restrictionNode.getParentNode(),], self.xsdTypeDict)
                    if not facetNsName:
                        continue
                    if _localFacetDict.has_key(facetNsName):
                        suppFacets = _localFacetDict[facetNsName]
                    else:
                        suppFacets, dummy, dummy = self.xsdTypeDict[facetNsName].getXPathList (".//hfp:hasFacet/@name" % vars())

                    specifiedFacets = {"length":None, "minLength":None, "maxLength":None,
                                       "minExclusive":None, "minInclusive":None, "maxExclusive":None, "maxInclusive":None,
                                       "totalDigits": None, "fractionDigits":None}
                    for childNode in restrictionNode.getChildren():
                        if childNode.getLocalName() in suppFacets:
                            if specifiedFacets.has_key(childNode.getLocalName()):
                                specifiedFacets[childNode.getLocalName()] = childNode["value"]
                            facetElementNode = self.xsdElementDict[childNode.getNsName()]
                            try:
                                self._checkElementTag (facetElementNode, restrictionNode, (childNode,), 0)
                            except TagException, errInst:
                                self._addError (errInst.errstr, errInst.node, errInst.endTag)
                            if childNode.getLocalName() in ("enumeration", "minExclusive", "minInclusive", "maxExclusive", "maxInclusive"):
                                simpleTypeReturnDict = self._checkSimpleType (restrictionNode, "base", childNode, "value", childNode["value"], None, checkAttribute=1)
                                if simpleTypeReturnDict != None and simpleTypeReturnDict.has_key("orderedValue"):
                                    if childNode.getLocalName() != "enumeration":
                                        specifiedFacets[childNode.getLocalName()] = simpleTypeReturnDict["orderedValue"]
                        elif childNode.getLocalName() == "enumeration":
                            self._checkSimpleType (restrictionNode, "base", childNode, "value", childNode["value"], None, checkAttribute=1)
                        elif childNode.getLocalName() != "annotation":
                            self._addError ("Facet %s not allowed for base type %s!" %(childNode.getLocalName(), repr(restrictionNode["base"])), childNode)
                    if specifiedFacets["length"] != None:
                        if specifiedFacets["minLength"] != None or specifiedFacets["maxLength"] != None:
                            self._addError ("Facet 'minLength' and 'maxLength' not allowed if facet 'length' is specified!", restrictionNode)
                    else:
                        if specifiedFacets["maxLength"] != None and specifiedFacets["minLength"] != None:
                            if int(specifiedFacets["maxLength"]) < int(specifiedFacets["minLength"]):
                                self._addError ("Facet 'maxLength' < facet 'minLength'!", restrictionNode)

                    if specifiedFacets["totalDigits"] != None and specifiedFacets["fractionDigits"] != None:
                        if int(specifiedFacets["totalDigits"]) < int(specifiedFacets["fractionDigits"]):
                            self._addError ("Facet 'totalDigits' must be >= 'fractionDigits'!", restrictionNode)

                    if specifiedFacets["minExclusive"] != None and specifiedFacets["minInclusive"] != None:
                        self._addError ("Facets 'minExclusive' and 'minInclusive' are mutually exclusive!", restrictionNode)
                    if specifiedFacets["maxExclusive"] != None and specifiedFacets["maxInclusive"] != None:
                        self._addError ("Facets 'maxExclusive' and 'maxInclusive' are mutually exclusive!", restrictionNode)

                    minValue = specifiedFacets["minExclusive"]
                    if specifiedFacets["minInclusive"] != None:
                        minValue = specifiedFacets["minInclusive"]
                    maxValue = specifiedFacets["maxExclusive"]
                    if specifiedFacets["maxInclusive"] != None:
                        maxValue = specifiedFacets["maxInclusive"]
                    # TODO: use orderedValue for '<' check!!
                    if minValue != None and maxValue != None and maxValue < minValue:
                        self._addError ("maxValue facet < minValue facet!", restrictionNode)

            except TagException:
                self._addError ("Primitive type for base type not found!", restrictionNode)

        listNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)slist" % vars())
        for listNode in listNodes:
            # check for unique ID
            self._checkNodeId (listNode)

            if not listNode.hasAttribute("itemType") and listNode.getFirstChildNS (self.inputNsURI, "simpleType") == None:
                self._addError ("List type must have 'itemType' attribute or 'simpleType' child tag!", listNode)
            elif listNode.hasAttribute("itemType") and listNode.getFirstChildNS (self.inputNsURI, "simpleType") != None:
                self._addError ("List type must not have 'itemType' attribute and 'simpleType' child tag!", listNode)
            elif listNode.hasAttribute("itemType"):
                itemType = self._checkType(listNode, "itemType", self.xsdTypeDict)
                if self.xsdTypeDict.has_key(itemType):
                    if self.xsdTypeDict[itemType].getLocalName() != "simpleType":
                        self._addError ("ItemType %s must be a simple type!" %(repr(itemType)), listNode)
                    elif self.xsdTypeDict[itemType].getFirstChild().getLocalName() == "list":
                        self._addError ("ItemType %s must not be a list type!" %(repr(itemType)), listNode)

        unionNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)ssimpleType/%(prefix)sunion" % vars())
        for unionNode in unionNodes:
            # check for unique ID
            self._checkNodeId (unionNode)

            if not unionNode.hasAttribute("memberTypes"):
                for childNode in unionNode.getChildren():
                    if childNode.getLocalName() != "annotation":
                        break
                else:
                    self._addError ("Union must not be empty!", unionNode)
            else:
                for memberType in string.split(unionNode["memberTypes"]):
                    memberNsName = unionNode.qName2NsName(memberType, 1)
                    self._checkBaseType(unionNode, memberNsName, self.xsdTypeDict)
                    if self.xsdTypeDict.has_key(memberNsName):
                        if self.xsdTypeDict[memberNsName].getLocalName() != "simpleType":
                            self._addError ("MemberType %s must be a simple type!" %(repr(memberNsName)), unionNode)

        patternNodes, dummy, dummy = self.inputRoot.getXPathList (".//%(prefix)spattern" % vars())
        for patternNode in patternNodes:
            pattern = patternNode["value"]
            try:
                pattern = substituteSpecialEscChars (pattern)
                try:
                    test = re.compile(pattern)
                except Exception, errstr:
                    self._addError (str(errstr), patternNode)
                    self._addError ("%s is not a valid regular expression!" %(repr(patternNode["value"])), patternNode)
            except SyntaxError, errInst:
                    self._addError (repr(errInst[0]), patternNode)


    ########################################
    # additional checks for keyrefs
    #
    def _checkIdentityConstraintsSecondLevel(self):
        identityConstraintNodes, dummy, dummy = self.inputRoot.getXPathList (".//%sunique" %(self.inputNsPrefixString))
        for identityConstraintNode in identityConstraintNodes:
            # check for unique ID
            self._checkNodeId (identityConstraintNode)

            selectorNode = identityConstraintNode.getFirstChildNS(XSD_NAMESPACE, "selector")
            self._checkNodeId (selectorNode)
            try:
                completeChildList, attrNodeList, attrNsNameFirst = identityConstraintNode.getParentNode().getXPathList (selectorNode["xpath"], selectorNode)
                if attrNsNameFirst != None:
                    self._addError ("Selection of attributes is not allowed for selector!", selectorNode)
            except Exception, errstr:
                self._addError (errstr, selectorNode)

            try:
                fieldNode = identityConstraintNode.getFirstChildNS(XSD_NAMESPACE, "field")
                identityConstraintNode.getParentNode().getXPathList (fieldNode["xpath"], fieldNode)
                self._checkNodeId (fieldNode)
            except Exception, errstr:
                self._addError (errstr, fieldNode)


    ########################################
    # additional checks for keyrefs
    #
    def _checkKeysSecondLevel(self):
        keyNodes, dummy, dummy = self.inputRoot.getXPathList (".//%skey" %(self.inputNsPrefixString))
        for keyNode in keyNodes:
            # check for unique ID
            self._checkNodeId (keyNode)

            fieldNode = keyNode.getFirstChildNS(XSD_NAMESPACE, "field")
            if fieldNode != None:
                self._checkNodeId (fieldNode)
                

    ########################################
    # additional checks for keyrefs
    #
    def _checkKeyRefsSecondLevel(self):
        keyrefNodes, dummy, dummy = self.inputRoot.getXPathList (".//%skeyref" %(self.inputNsPrefixString))
        for keyrefNode in keyrefNodes:
            # check for unique ID
            self._checkNodeId (keyrefNode)

            self._checkKeyRef(keyrefNode, self.xsdIdentityConstrDict)
                

    ########################################
    # helper methods
    #

    def _checkFixedDefault(self, node):
        if node.hasAttribute("default") and node.hasAttribute("fixed"):
            self._addError ("%s may have 'default' OR 'fixed' attribute!" %repr(node.getLocalName()), node)
        if  node.hasAttribute("default"):
            self._checkSimpleType (node, "type", node, "default", node["default"], None, checkAttribute=1)
        if  node.hasAttribute("fixed"):
            self._checkSimpleType (node, "type", node, "fixed", node["fixed"], None, checkAttribute=1)
    
    
    def _checkReference(self, node, dict):
        baseNsName = node.getQNameAttribute("ref")
        if dict.has_key(baseNsName):
            refNode = dict[baseNsName]
            fixedValue = node.getAttribute("fixed")
            fixedRefValue = refNode.getAttribute("fixed")
            if fixedValue != None and fixedRefValue != None and fixedValue != fixedRefValue:
                self._addError ("Fixed value %s of attribute does not match fixed value %s of reference!" %(repr(fixedValue), repr(fixedRefValue)), node)
                
        else:
            self._addError ("Reference %s not found!" %(repr(baseNsName)), node)

    def _checkType(self, node, typeAttrName, dict, typeNsName=None):
        baseNsName = node.getQNameAttribute(typeAttrName)
        self._checkBaseType(node, baseNsName, dict, typeNsName)
        return baseNsName
    
    def _checkBaseType(self, node, baseNsName, dict, typeNsName=None):
        if not dict.has_key(baseNsName) and baseNsName != (XSD_NAMESPACE, "anySimpleType"):
            self._addError ("Definition of type %s not found!" %(repr(baseNsName)), node)
        elif typeNsName != None:
            if typeNsName == (XSD_NAMESPACE, "simpleContent"):
                if node.getNsName() == (XSD_NAMESPACE, "restriction"):
                    if (baseNsName != (XSD_NAMESPACE, "anySimpleType") and
                        dict[baseNsName].getNsName() == (XSD_NAMESPACE, "complexType") and
                        dict[baseNsName].getFirstChild().getNsName() == typeNsName):
                        pass
                    else:
                        self._addError ("Referred type %s must be a complex type with simple content!" %(repr(baseNsName)), node)
                else: # extension
                    if (baseNsName == (XSD_NAMESPACE, "anySimpleType") or
                        dict[baseNsName].getNsName() == (XSD_NAMESPACE, "simpleType") or
                       (dict[baseNsName].getNsName() == (XSD_NAMESPACE, "complexType") and
                        dict[baseNsName].getFirstChild().getNsName() == typeNsName)):
                        pass
                    else:
                        self._addError ("Referred type %s must be a simple type or a complex type with simple content!" %(repr(baseNsName)), node)
            else:
                if typeNsName == (XSD_NAMESPACE, "simpleType") and baseNsName == (XSD_NAMESPACE, "anySimpleType"):
                    pass
                elif dict[baseNsName].getNsName() != typeNsName:
                    self._addError ("Referred type %s must be a %s!" %(repr(baseNsName), repr(typeNsName)), node)


    def _checkKeyRef(self, keyrefNode, dict):
        baseNsName = keyrefNode.getQNameAttribute("refer")
        if not dict.has_key(baseNsName):
            self._addError ("keyref refers unknown key %s!" %(repr(baseNsName)), keyrefNode)
        else:
            keyNode = dict[baseNsName]["Node"]
            if keyNode.getNsName() not in ((XSD_NAMESPACE, "key"), (XSD_NAMESPACE, "unique")):
                self._addError ("reference to non-key constraint %s!" %(repr(baseNsName)), keyrefNode)
            if len(keyrefNode.getChildrenNS(XSD_NAMESPACE, "field")) != len(keyNode.getChildrenNS(XSD_NAMESPACE, "field")):
                self._addError ("key/keyref field size mismatch!", keyrefNode)
                
            
    def _checkOccurs (self, node):
        minOccurs = node.getAttributeOrDefault("minOccurs", "1")
        maxOccurs = node.getAttributeOrDefault("maxOccurs", "1")
        if maxOccurs != "unbounded":
            if string.atoi(minOccurs) > string.atoi(maxOccurs):
                self._addError ("Attribute minOccurs > maxOccurs!", node)


    def _checkNodeId (self, node, unambiguousPerFile=1):
        if node.hasAttribute("id"):
            # id must only be unambiguous within one file
            if unambiguousPerFile:
                nodeId = (node.getAbsUrl(), collapseString(node["id"]))
            else:
                nodeId = collapseString(node["id"])
            if not self.xsdIdDict.has_key(nodeId):
                self.xsdIdDict[nodeId] = node
            else:
                self._addError ("There are multiple occurences of ID value %s!" %repr(nodeId), node)


    def _getFacetType(self, node, parentNodeList, xsdTypeDict):
            baseNsName = node.getQNameAttribute("base")
            try:
                baseNode = xsdTypeDict[baseNsName]
            except:
                self._addError ("Base type %s must be an atomic simple type definition or a builtin type!" %repr(baseNsName), node)
                return None

            if baseNode in parentNodeList:
                self._addError ("Circular type definition (type is contained in its own type hierarchy)!", node)
                return None
                
            if baseNode.getNsName() == (XSD_NAMESPACE, "simpleType"):
                if baseNode.getAttribute("facetType") != None:
                    facetType = baseNode.qName2NsName(baseNode["facetType"], 1)
                    node.getParentNode()["facetType"] = node.nsName2QName(facetType)
                    return facetType
                else:
                    for baseNodeType in ("list", "union"):
                        if baseNode.getFirstChildNS (XSD_NAMESPACE, baseNodeType) != None:
                            return (XSD_NAMESPACE, baseNodeType)
                    else:
                        parentNodeList.append(node)
                        return self._getFacetType(baseNode.getFirstChildNS(XSD_NAMESPACE, "restriction"), parentNodeList, xsdTypeDict)    
            else:
                self._addError ("Base type %s must be an atomic simple type definition or a builtin type!" %repr(baseNsName), node)
                return None
            

