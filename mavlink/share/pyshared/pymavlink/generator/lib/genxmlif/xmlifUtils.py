#
# genxmlif, Release 0.9.0
# file: xmlifUtils.py
#
# utility module for genxmlif
#
# history:
# 2005-04-25 rl   created
# 2008-08-01 rl   encoding support added
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
import re
import os
import urllib
import urlparse
from types   import StringTypes, TupleType
from xml.dom import EMPTY_PREFIX, EMPTY_NAMESPACE

######################################################################
# DEFINITIONS
######################################################################

######################################################################
# REGULAR EXPRESSION OBJECTS
######################################################################

_reWhitespace  = re.compile('\s')
_reWhitespaces = re.compile('\s+')

_reSplitUrlApplication = re.compile (r"(file|http|ftp|gopher):(.+)") # "file:///d:\test.xml" => "file" + "///d:\test.xml"


######################################################################
# FUNCTIONS
######################################################################


########################################
# remove all whitespaces from a string
#
def removeWhitespaces (strValue):
    return _reWhitespaces.sub('', strValue)


########################################
# substitute multiple whitespace characters by a single ' '
#
def collapseString (strValue, lstrip=1, rstrip=1):
    collStr = _reWhitespaces.sub(' ', strValue)
    if lstrip and rstrip:
        return collStr.strip()
    elif lstrip:
        return collStr.lstrip()
    elif rstrip:
        return collStr.rstrip()
    else:
        return collStr
        


########################################
# substitute each whitespace characters by a single ' '
#
def normalizeString (strValue):
    return _reWhitespace.sub(' ', strValue)


########################################
# process whitespace action
#
def processWhitespaceAction (strValue, wsAction, lstrip=1, rstrip=1):
    if wsAction == "collapse":
        return collapseString(strValue, lstrip, rstrip)
    elif wsAction == "replace":
        return normalizeString(strValue)
    else:
        return strValue
    

##########################################################
#  convert input parameter 'fileOrUrl' into a valid URL

def convertToUrl (fileOrUrl):
    matchObject = _reSplitUrlApplication.match(fileOrUrl)
    if matchObject:
        # given fileOrUrl is an absolute URL
        if matchObject.group(1) == 'file':
            path = re.sub(':', '|', matchObject.group(2)) # replace ':' by '|' in the path string
            url = "file:" + path
        else:
            url = fileOrUrl
    elif not os.path.isfile(fileOrUrl):
        # given fileOrUrl is treated as a relative URL
        url = fileOrUrl
    else:
        # local filename
#        url = "file:" + urllib.pathname2url (fileOrUrl)
        url = urllib.pathname2url (fileOrUrl)

    return url


##########################################################
#  convert input parameter 'fileOrUrl' into a valid absolute URL

def convertToAbsUrl (fileOrUrl, baseUrl):
    if fileOrUrl == "" and baseUrl != "":
        absUrl = "file:" + urllib.pathname2url (os.path.join(os.getcwd(), baseUrl, "__NO_FILE__"))
    elif os.path.isfile(fileOrUrl):
        absUrl = "file:" + urllib.pathname2url (os.path.join(os.getcwd(), fileOrUrl))
    else:
        matchObject = _reSplitUrlApplication.match(fileOrUrl)
        if matchObject:
            # given fileOrUrl is an absolute URL
            if matchObject.group(1) == 'file':
                path = re.sub(':', '|', matchObject.group(2)) # replace ':' by '|' in the path string
                absUrl = "file:" + path
            else:
                absUrl = fileOrUrl
        else:
            # given fileOrUrl is treated as a relative URL
            if baseUrl != "":
                absUrl = urlparse.urljoin (baseUrl, fileOrUrl)
            else:
                absUrl = fileOrUrl
#                raise IOError, "File %s not found!" %(fileOrUrl)
    return absUrl


##########################################################
#  normalize filter
def normalizeFilter (filterVar):
    if filterVar == None or filterVar == '*':
        filterVar = ("*",)
    elif not isinstance(filterVar, TupleType):
        filterVar = (filterVar,)
    return filterVar


######################################################################
# Namespace handling
######################################################################

def nsNameToQName (nsLocalName, curNs):
    """Convert a tuple '(namespace, localName)' to a string 'prefix:localName'
    
    Input parameter:
        nsLocalName:   tuple '(namespace, localName)' to be converted
        curNs:         list of current namespaces
    Returns the corresponding string 'prefix:localName' for 'nsLocalName'.
    """
    ns = nsLocalName[0]
    for prefix, namespace in curNs:
        if ns == namespace:
            if prefix != None:
                return "%s:%s" %(prefix, nsLocalName[1])
            else:
                return "%s" %nsLocalName[1]
    else:
        if ns == None:
            return nsLocalName[1]
        else:
            raise LookupError, "Prefix for namespaceURI '%s' not found!" % (ns)


def splitQName (qName):
    """Split the given 'qName' into prefix/namespace and local name.

    Input parameter:
        'qName':  contains a string 'prefix:localName' or '{namespace}localName'
    Returns a tuple (prefixOrNamespace, localName)
    """
    namespaceEndIndex = string.find (qName, '}')
    if namespaceEndIndex != -1:
        prefix     = qName[1:namespaceEndIndex]
        localName  = qName[namespaceEndIndex+1:]
    else:
        namespaceEndIndex = string.find (qName, ':')
        if namespaceEndIndex != -1:
            prefix     = qName[:namespaceEndIndex]
            localName  = qName[namespaceEndIndex+1:]
        else:
            prefix     = None
            localName  = qName
    return prefix, localName


def toClarkQName (tupleOrLocalName):
    """converts a tuple (namespace, localName) into clark notation {namespace}localName
       qNames without namespace remain unchanged

    Input parameter:
        'tupleOrLocalName':  tuple '(namespace, localName)' to be converted
    Returns a string {namespace}localName
    """
    if isinstance(tupleOrLocalName, TupleType):
        if tupleOrLocalName[0] != EMPTY_NAMESPACE:
            return "{%s}%s" %(tupleOrLocalName[0], tupleOrLocalName[1])
        else:
            return tupleOrLocalName[1]
    else:
        return tupleOrLocalName
    
    
def splitClarkQName (qName):
    """converts clark notation {namespace}localName into a tuple (namespace, localName)

    Input parameter:
        'qName':  {namespace}localName to be converted
    Returns prefix and localName as separate strings
    """
    namespaceEndIndex = string.find (qName, '}')
    if namespaceEndIndex != -1:
        prefix     = qName[1:namespaceEndIndex]
        localName  = qName[namespaceEndIndex+1:]
    else:
        prefix     = None
        localName  = qName
    return prefix, localName
    
    
##################################################################
# XML serialization of text
# the following functions assume an ascii-compatible encoding
# (or "utf-16")

_escape = re.compile(eval(r'u"[&<>\"\u0080-\uffff]+"'))

_escapeDict = {
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
}


def _raiseSerializationError(text):
    raise TypeError("cannot serialize %r (type %s)" % (text, type(text).__name__))


def _encode(text, encoding):
    try:
        return text.encode(encoding)
    except AttributeError:
        return text # assume the string uses the right encoding


def _encodeEntity(text, pattern=_escape):
    # map reserved and non-ascii characters to numerical entities
    def escapeEntities(m, map=_escapeDict):
        out = []
        append = out.append
        for char in m.group():
            text = map.get(char)
            if text is None:
                text = "&#%d;" % ord(char)
            append(text)
        return string.join(out, "")
    try:
        return _encode(pattern.sub(escapeEntities, text), "ascii")
    except TypeError:
        _raise_serialization_error(text)


def escapeCdata(text, encoding=None, replace=string.replace):
    # escape character data
    try:
        if encoding:
            try:
                text = _encode(text, encoding)
            except UnicodeError:
                return _encodeEntity(text)
        text = replace(text, "&", "&amp;")
        text = replace(text, "<", "&lt;")
        text = replace(text, ">", "&gt;")
        return text
    except (TypeError, AttributeError):
        _raiseSerializationError(text)


def escapeAttribute(text, encoding=None, replace=string.replace):
    # escape attribute value
    try:
        if encoding:
            try:
                text = _encode(text, encoding)
            except UnicodeError:
                return _encodeEntity(text)
        text = replace(text, "&", "&amp;")
        text = replace(text, "'", "&apos;") # FIXME: overkill
        text = replace(text, "\"", "&quot;")
        text = replace(text, "<", "&lt;")
        text = replace(text, ">", "&gt;")
        return text
    except (TypeError, AttributeError):
        _raiseSerializationError(text)


######################################################################
# CLASSES
######################################################################

######################################################################
# class containing a tuple of namespace prefix and localName
#
class QNameTuple(tuple):
    def __str__(self):
        if self[0] != EMPTY_PREFIX:
            return "%s:%s" %(self[0],self[1])
        else:
            return self[1]
    

def QNameTupleFactory(initValue):
    if isinstance(initValue, StringTypes):
        separatorIndex = string.find (initValue, ':')
        if separatorIndex != -1:
            initValue = (initValue[:separatorIndex], initValue[separatorIndex+1:])
        else:
           initValue = (EMPTY_PREFIX, initValue)
    return QNameTuple(initValue)


######################################################################
# class containing a tuple of namespace and localName
#
class NsNameTuple(tuple):
    def __str__(self):
        if self[0] != EMPTY_NAMESPACE:
            return "{%s}%s" %(self[0],self[1])
        elif self[1] != None:
            return self[1]
        else:
            return "None"


def NsNameTupleFactory(initValue):
    if isinstance(initValue, StringTypes):
        initValue = splitClarkQName(initValue)
    elif initValue == None:
        initValue = (EMPTY_NAMESPACE, initValue)
    return NsNameTuple(initValue)


