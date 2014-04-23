#!/usr/local/bin/python

import sys
import getopt
from ..genxmlif          import GenXmlIfError
from xsvalErrorHandler import ErrorHandler, XsvalError
from ..minixsv           import *
from pyxsval           import parseAndValidate


##########################################
# minixsv Wrapper for calling minixsv from command line

validSyntaxText = '''\
minixsv XML Schema Validator
Syntax: minixsv [-h] [-?] [-p Parser] [-s XSD-Filename] XML-Filename

Options:
-h, -?:          Display this help text
-p Parser:       XML Parser to be used 
                 (XMLIF_MINIDOM, XMLIF_ELEMENTTREE, XMLIF_4DOM
                  default: XMLIF_ELEMENTTREE)
-s XSD-FileName: specify the schema file for validation 
                 (if not specified in XML-File)
'''

def checkShellInputParameter():
    """check shell input parameters."""
    xmlInputFilename = None
    xsdFilename = None
    xmlParser = "XMLIF_ELEMENTTREE"
    try:
        (options, arguments) = getopt.getopt(sys.argv[1:], '?hp:s:')

        if ('-?','') in options or ('-h','') in options:
            print validSyntaxText
            sys.exit(-1)
        else:
            if len (arguments) == 1:
                xmlInputFilename = arguments[0]
                for o, a in options:
                    if o == "-s":
                        xsdFilename = a
                    if o == "-p":
                        if a in (XMLIF_MINIDOM, XMLIF_ELEMENTTREE, XMLIF_4DOM):
                            xmlParser = a    
                        else:
                            print 'Invalid XML parser %s!' %(a)
                            sys.exit(-1)
            else:
                print 'minixsv needs one argument (XML input file)!'
                sys.exit(-1)

    except getopt.GetoptError, errstr:
        print errstr
        sys.exit(-1)
    return xmlInputFilename, xsdFilename, xmlParser


def main():
    xmlInputFilename, xsdFileName, xmlParser = checkShellInputParameter()
    try:
        parseAndValidate (xmlInputFilename, xsdFile=xsdFileName, xmlIfClass=xmlParser)
    except IOError, errstr:
        print errstr
        sys.exit(-1)
    except GenXmlIfError, errstr:
        print errstr
        sys.exit(-1)
    except XsvalError, errstr:
        print errstr
        sys.exit(-1)
    
if __name__ == "__main__":
    main()
    
