Release Notes for minixsv, Release 0.9.0

minixsv is a XML schema validator written in "pure" Python
(minixsv requires at least Python 2.4)

Currently a subset of the XML schema standard is supported
(list of limitations/restrictions see below).

minixsv is based on genxmlif, a generic XML interface class,
which currently supports minidom, elementtree or 4DOM/pyXML as XML parser
Other parsers can be adapted by implementing an appropriate derived interface class

Using the 4DOM interface is rather slow. 
For best performance the elementtree parser should be used!

After successful validation minixsv provides the input XML tree with the following changes:
- Whitespaces inside strings are automatically normalized/collapsed as specified
  in the XML schema file
- Default/Fixed attributes are automatically inserted if not specified in the input file
- The "post validation" XML tree can be accessed using genxmlif or the contained
  original interface (minidom, elementtree or 4DOM/pyXML).

--------------------------------------------------------------------
 The minixsv XML schema validator is

 Copyright (c) 2004-2008 by Roland Leuthe

 By obtaining, using, and/or copying this software and/or its
 associated documentation, you agree that you have read, understood,
 and will comply with the following terms and conditions:

 Permission to use, copy, modify, and distribute this software and
 its associated documentation for any purpose and without fee is
 hereby granted, provided that the above copyright notice appears in
 all copies, and that both that copyright notice and this permission
 notice appear in supporting documentation, and that the name of
 the author not be used in advertising or publicity
 pertaining to distribution of the software without specific, written
 prior permission.

 THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD
 TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANT-
 ABILITY AND FITNESS.  IN NO EVENT SHALL THE AUTHOR
 BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY
 DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 OF THIS SOFTWARE.
---------------------------------------------------------------------


Limitations/restrictions of the current release:

- no checks if derived type and base type match
- no check of attributes "final", "finalDefault"
- no support of substitution groups
- no support of abstract elements and types
- restrictions regarding for pattern matching:
  * subtraction of character sets not supported, e.g. regex = "[\w-[ab]]"
  * characters sets with \I, \C, \P{...} not supported, e.g. regex = "[\S\I\?a-c\?]"
    (character sets with \i, \c, \p{...} are supported!)

Note: This constraint list may not be complete!!


---------------------------------------------------------------------

Contents
========

README.txt
__init__.py
minixsv
minixsvWrapper.py
pyxsval.py
xsvalBase.py
xsvalErrorHandler.py
xsvalSchema.py
xsvalSimpleTypes.py
xsvalUtils.py
xsvalXmlIf.py
datatypes.xsd
XInclude.xsd
xml.xsd
XMLSchema.xsd
XMLSchema-instance.xsd

---------------------------------------------------------------------

HISTORY:
=======

Changes for Release 0.9.0
=========================

- Caution, Interface changed! 
  * In case of parser and XInclude errors now a GenXmlIfError exception is raised!
  * New optional parameter 'useCaching=1' and 'processXInclude=1' to XsValidator class added
  
- check of facets of derived primitive types added
- unicode support added (except wide unicode characters)
- major improvements for pattern matching (but there are still some restrictions, refer above)
- limited support of XInclude added (no support of fallback tag)
- performance optimizations (caching introduced)
- several bugs fixed (new W3C test suite (2006-11-06) passed for supported features)
  3943 of 3953 nisttest  testgroups passed
  8645 of 9745 mstest    testgroups passed
   559 of  679 suntest   testgroups passed
  (most not passed test groups correspond to the limitations listed above)


Changes for Release 0.8
=======================

- Caution, Interface changed! 
  When calling the validator (e.g. using parseAndValidate or parseAndValidateString)
  the input parameter xsdFile/xsdText is only used if no schema file is specified in the XML input file/string,
  i.e. the schema specification in the XML input file/string has now priority!!
  
- uniqueness of attribute/attributeGroup/type IDs now checked
- problems with different target namespaces fixed
- "redefine" element now supported
- performance optimization due to optimized genxmlif (when elementtree parser is used)
- several bugs fixed (W3C test suite passed for supported features)
  3953 of 3953 nisttest  testgroups passed
  4260 of 4529 msxsdtest testgroups passed
    31 of   40 suntest   testgroups passed
  (most not passed test groups correspond to the limitations listed above)


Changes for Release 0.7
=======================

- now all primitive data types are supported
- check if no element content exists when xsi:nil = "true" is specified
- support of "processContents" attribute
- correct uniqueness checking of identity constraints (unique/key/keyref)
- many bugs fixed (W3C test suite passed for supported features)
  3953 of 3953 nisttest  testgroups passed
  3996 of 4529 msxsdtest testgroups passed
    27 of   40 suntest   testgroups passed
  (most not passed test groups correspond to the limitations listed above)
  

Changes for Release 0.5
=======================

- generic XML interface extracted into a separate python package
- namespace support added
- 4DOM support added
- support of attributes "elementFormDefault", "attributeFormDefault", "form" added
- support of "import" element added
- handling of "schemaLocation" and "noNamespaceSchemaLocation" corrected
- support of derivation of complex types from simple types (extension) added
- support of mixed content (child nodes and text nodes) added
- new access function to add user defined XML interface class
- new access function to add user defined validation function for unsupported predefined types
- several bugs fixed


Changes for Release 0.3
=======================

- API re-structured
- XML text processing added
- several bugs fixed
- internal re-structuring


Changes for Release 0.2
=======================

- Error/warning outputs contain now also filename and linenumber
- Basic URI support for include directive added
- XML interface classes completely re-designed
- several bugs fixed


