#!/usr/bin/env python

'''
parse a MAVLink protocol XML file and generate a python implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

'''
import sys, textwrap, os
try:
    import mavparse
except Exception:
    from pymavlink.generator import mavparse

try:
    from lib.genxmlif import GenXmlIfError
    from lib.minixsv import pyxsval
    performValidation = True
except Exception:
    print("Unable to load XML validator libraries. XML validation will not be performed")
    performValidation = False
    
# XSD schema file
schemaFile = os.path.join(os.path.dirname(os.path.realpath(__file__)), "mavschema.xsd")


def mavgen(opts, args) :
    """Generate mavlink message formatters and parsers (C and Python ) using options
    and args where args are a list of xml files. This function allows python
    scripts under Windows to control mavgen using the same interface as
    shell scripts under Unix"""

    xml = []

    for fname in args:
        if performValidation:
            print("Validating %s" % fname)
            mavgen_validate(fname, schemaFile, opts.error_limit);
        else:
            print("Validation skipped for %s." % fname)

        print("Parsing %s" % fname)
        xml.append(mavparse.MAVXML(fname, opts.wire_protocol))

    # expand includes
    for x in xml[:]:
        for i in x.include:
            fname = os.path.join(os.path.dirname(x.filename), i)

            ## Validate XML file with XSD file if possible.
            if performValidation:
                print("Validating %s" % fname)
                mavgen_validate(fname, schemaFile, opts.error_limit);
            else:
                print("Validation skipped for %s." % fname)

            ## Parsing
            print("Parsing %s" % fname)
            xml.append(mavparse.MAVXML(fname, opts.wire_protocol))

            # include message lengths and CRCs too
            for idx in range(0, 256):
                if x.message_lengths[idx] == 0:
                    x.message_lengths[idx] = xml[-1].message_lengths[idx]
                    x.message_crcs[idx] = xml[-1].message_crcs[idx]
                    x.message_names[idx] = xml[-1].message_names[idx]

    # work out max payload size across all includes
    largest_payload = 0
    for x in xml:
        if x.largest_payload > largest_payload:
            largest_payload = x.largest_payload
    for x in xml:
        x.largest_payload = largest_payload

    if mavparse.check_duplicates(xml):
        sys.exit(1)

    print("Found %u MAVLink message types in %u XML files" % (
        mavparse.total_msgs(xml), len(xml)))

    # Convert language option to lowercase and validate
    opts.language = opts.language.lower()
    if opts.language == 'python':
        import mavgen_python
        mavgen_python.generate(opts.output, xml)
    elif opts.language == 'c':
        try:
            import mavgen_c
        except Exception:
            from pymavlink.generator import mavgen_c
        mavgen_c.generate(opts.output, xml)
    elif opts.language == 'wlua':
        import mavgen_wlua
        mavgen_wlua.generate(opts.output, xml)
    elif opts.language == 'cs':
        import mavgen_cs
        mavgen_cs.generate(opts.output, xml)
    elif opts.language == 'javascript':
        import mavgen_javascript
        mavgen_javascript.generate(opts.output, xml)
    else:
        print("Unsupported language %s" % opts.language)


# build all the dialects in the dialects subpackage
class Opts:
    def __init__(self, wire_protocol, output):
        self.wire_protocol = wire_protocol
        self.error_limit = 200
        self.language = 'Python'
        self.output = output

def mavgen_python_dialect(dialect, wire_protocol):
    '''generate the python code on the fly for a MAVLink dialect'''
    dialects = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'dialects')
    mdef = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'message_definitions')
    if wire_protocol == mavparse.PROTOCOL_0_9:
        py = os.path.join(dialects, 'v09', dialect + '.py')
        xml = os.path.join(dialects, 'v09', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v0.9', dialect + '.xml')
    else:
        py = os.path.join(dialects, 'v10', dialect + '.py')
        xml = os.path.join(dialects, 'v10', dialect + '.xml')
        if not os.path.exists(xml):
            xml = os.path.join(mdef, 'v1.0', dialect + '.xml')
    opts = Opts(wire_protocol, py)
    import StringIO

    # throw away stdout while generating
    stdout_saved = sys.stdout
    sys.stdout = StringIO.StringIO()
    try:
        xml = os.path.relpath(xml)
        mavgen( opts, [xml] )
    except Exception:
        sys.stdout = stdout_saved
        raise
    sys.stdout = stdout_saved
    

def mavgen_validate(fname, schema, errorLimitNumber) :
    """Uses minixsv to validate an XML file with a given XSD schema file."""
    # use default values of minixsv, location of the schema file must be specified in the XML file
    domTreeWrapper = pyxsval.parseAndValidate(fname, xsdFile=schema, errorLimit=errorLimitNumber)
            
    # domTree is a minidom document object
    domTree = domTreeWrapper.getTree()


if __name__ == "__main__":
    from optparse import OptionParser

    supportedLanguages = ["C", "CS", "JavaScript", "Python", "WLua"]
    parser = OptionParser("%prog [options] <XML files>")
    parser.add_option("-o", "--output", dest="output", default="mavlink", help="output directory.")
    parser.add_option("--lang", dest="language", choices=supportedLanguages, default="Python", help="language of generated code, one of: {0} [default: %default]".format(supportedLanguages))
    parser.add_option("--wire-protocol", dest="wire_protocol", choices=[mavparse.PROTOCOL_0_9, mavparse.PROTOCOL_1_0], default=mavparse.PROTOCOL_1_0, help="MAVLink protocol version: '0.9' or '1.0'. [default: %default]")
    parser.add_option("--error-limit", dest="error_limit", default=200, help="maximum number of validation errors.")
    (opts, args) = parser.parse_args()

    if len(args) < 1:
        parser.error("You must supply at least one MAVLink XML protocol definition")
    mavgen(opts, args)
