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

    def __init__(self, groups, board):
        xml_parameters = ET.Element("parameters")
        xml_version = ET.SubElement(xml_parameters, "version")
        xml_version.text = "3"
        xml_version = ET.SubElement(xml_parameters, "parameter_version_major")
        xml_version.text = "1"
        xml_version = ET.SubElement(xml_parameters, "parameter_version_minor")
        xml_version.text = "15"
        last_param_name = ""
        board_specific_param_set = False
        for group in groups:
            xml_group = ET.SubElement(xml_parameters, "group")
            xml_group.attrib["name"] = group.name
            if group.no_code_generation:
                xml_group.attrib["no_code_generation"] = group.no_code_generation
            for param in sorted(group.parameters):
                if (last_param_name == param.name and not board_specific_param_set) or last_param_name != param.name:
                    xml_param = ET.SubElement(xml_group, "parameter")
                    xml_param.attrib["name"] = param.name
                    xml_param.attrib["default"] = param.default
                    xml_param.attrib["type"] = param.type
                    if param.volatile:
                        xml_param.attrib["volatile"] = "true"
                    if param.boolean:
                        xml_param.attrib["boolean"] = "true"
                    if param.category != "":
                        xml_param.attrib["category"] = param.category.title()
                    last_param_name = param.name
                    for code in param.field_keys:
                        value = param.fields.get(code, "")
                        if code == "board":
                            if value == board:
                                board_specific_param_set = True
                                xml_field = ET.SubElement(xml_param, code)
                                xml_field.text = value
                            else:
                                xml_group.remove(xml_param)
                        else:
                            xml_field = ET.SubElement(xml_param, code)
                            xml_field.text = value
                if last_param_name != param.name:
                    board_specific_param_set = False
                
                if len(param.enum.keys()) > 0:
                    xml_values = ET.SubElement(xml_param, "values")
                    for code in sorted(param.enum.keys(), key=float):
                        xml_value = ET.SubElement(xml_values, "value")
                        xml_value.attrib["code"] = code;
                        xml_value.text = param.enum[code]

                if len(param.bitmask.keys()) > 0:
                    xml_values = ET.SubElement(xml_param, "bitmask")
                    for index in sorted(param.bitmask.keys(), key=float):
                        xml_value = ET.SubElement(xml_values, "bit")
                        xml_value.attrib["index"] = index;
                        xml_value.text = param.bitmask.get(index, "")

        indent(xml_parameters)
        self.xml_document = ET.ElementTree(xml_parameters)

    def Save(self, filename):
        self.xml_document.write(filename, encoding="UTF-8")
