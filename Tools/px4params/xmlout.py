import xml.etree.ElementTree as ET
import codecs

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

class XMLOutput():

    def __init__(self, groups):
        xml_parameters = ET.Element("parameters")
        xml_version = ET.SubElement(xml_parameters, "version")
        xml_version.text = "2"
        for group in groups:
            xml_group = ET.SubElement(xml_parameters, "group")
            xml_group.attrib["name"] = group.GetName()
            for param in group.GetParams():
                xml_param = ET.SubElement(xml_group, "parameter")
                for code in param.GetFieldCodes():
                    value = param.GetFieldValue(code)
                    if code == "code":
                        xml_param.attrib["name"] = value
                    elif code == "default":
                        xml_param.attrib["default"] = value
                    elif code == "type":
                        xml_param.attrib["type"] = value
                    else:
                        xml_field = ET.SubElement(xml_param, code)
                        xml_field.text = value
        indent(xml_parameters)
        self.xml_document = ET.ElementTree(xml_parameters)

    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            self.xml_document.write(f)
