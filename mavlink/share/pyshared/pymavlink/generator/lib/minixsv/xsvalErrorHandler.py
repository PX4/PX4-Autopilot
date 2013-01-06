#
# minixsv, Release 0.9.0
# file: xsvalErrorHandler.py
#
# XML schema validator classes
#
# history:
# 2004-09-23 rl   created
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
import os

IGNORE_WARNINGS   = 0
PRINT_WARNINGS    = 1
STOP_ON_WARNINGS  = 2


########################################
# Error-Handler class for XML schema validator
# handles only validator errors, no parser errors!

class ErrorHandler:

    def __init__(self, errorLimit, warningProc, verbose):
        self.errorLimit  = errorLimit
        self.warningProc = warningProc
        self.verbose     = verbose
        
        self.errorList = []
        self.noOfErrors = 0
        self.warningList = []
        self.infoDict = {}


    ########################################
    # check if errors have already been reported

    def hasErrors (self):
        return self.errorList != []

    ########################################
    # add error to errorList (raise exception only if error limit is reached)

    def addError (self, errstr, element=None, endTag=0):
        filePath = ""
        lineNo = 0
        if element:
            filePath = element.getFilePath()
            if endTag:
                lineNo = element.getEndLineNumber()
            else:
                lineNo = element.getStartLineNumber()
        self.errorList.append ((filePath, lineNo, "ERROR", "%s" %errstr))
        self.noOfErrors += 1
        if self.noOfErrors == self.errorLimit:
            self._raiseXsvalException ("\nError Limit reached!!")


    ########################################
    # add warning to warningList

    def addWarning (self, warnstr, element=None):
        filePath = ""
        lineNo = 0
        if element:
            filePath = element.getFilePath()
            lineNo = element.getStartLineNumber()
        self.warningList.append ((filePath, lineNo, "WARNING", warnstr))


    ########################################
    # add info string to errorList

    def addInfo (self, infostr, element=None):
        self.infoDict.setdefault("INFO: %s" %infostr, 1)


    ########################################
    # add error to errorList (if given) and raise exception

    def raiseError (self, errstr, element=None):
        self.addError (errstr, element)
        self._raiseXsvalException ()


    ########################################
    # raise exception with complete errorList as exception string
    # (only if errors occurred)

    def flushOutput (self):
        if self.infoDict != {}:
            print string.join (self.infoDict.keys(), "\n")
            self.infoList = []

        if self.warningProc == PRINT_WARNINGS and self.warningList != []:
            print self._assembleOutputList(self.warningList, sorted=1)
            self.warningList = []
        elif self.warningProc == STOP_ON_WARNINGS:
            self.errorList.extend (self.warningList)

        if self.errorList != []:
            self._raiseXsvalException ()


    ########################################
    # Private methods

    def _raiseXsvalException (self, additionalInfo=""):
        output = self._assembleOutputList(self.errorList) + additionalInfo
        self.errorList = self.warningList = []
        raise XsvalError (output)


    def _assembleOutputList (self, outputList, sorted=0):
        if sorted:
            outputList.sort()
        outputStrList = []
        for outElement in outputList:
            outputStrList.append (self._assembleOutString(outElement))
        return string.join (outputStrList, "\n")
        
        
    def _assembleOutString (self, listElement):
        fileStr = ""
        lineStr = ""
        if listElement[0] != "":
            if self.verbose:
                fileStr = "%s: " %(listElement[0])
            else:
                fileStr = "%s: " %(os.path.basename(listElement[0]))
        if listElement[1] != 0:
            lineStr = "line %d: " %(listElement[1])
        return "%s: %s%s%s" %(listElement[2], fileStr, lineStr, listElement[3])
    

class XsvalError (StandardError):
    pass

