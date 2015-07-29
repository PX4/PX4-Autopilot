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
        xml_parameters = ET.Element("airframes")
        xml_version = ET.SubElement(xml_parameters, "version")
        xml_version.text = "1"
        last_param_name = ""
        board_specific_param_set = False
        for group in groups:
            xml_group = ET.SubElement(xml_parameters, "airframe_group")
            xml_group.attrib["name"] = group.GetName()
            if (group.GetName() == "Standard Plane"):
                xml_group.attrib["image"] = "AirframeStandardPlane.png"
            elif (group.GetName() == "Flying Wing"):
                xml_group.attrib["image"] = "AirframeFlyingWing.png"
            elif (group.GetName() == "Quadrotor x"):
                xml_group.attrib["image"] = "AirframeQuadRotorX.png"
            elif (group.GetName() == "Quadrotor +"):
                xml_group.attrib["image"] = "AirframeQuadRotorPlus.png"
            elif (group.GetName() == "Hexa x"):
                xml_group.attrib["image"] = "AirframeHexaRotorX.png"
            elif (group.GetName() == "Hexa +"):
                xml_group.attrib["image"] = "AirframeHexaRotorPlus.png"
            elif (group.GetName() == "Octo +"):
                xml_group.attrib["image"] = "AirframeOctoRotorPlus.png"
            elif (group.GetName() == "Octo X"):
                xml_group.attrib["image"] = "AirframeOctoRotorX.png"
            elif (group.GetName() == "Quadrotor H"):
                xml_group.attrib["image"] = "AirframeQuadRotorH.png"
            elif (group.GetName() == "Simulation"):
                xml_group.attrib["image"] = "AirframeSimulation.png"
            else:
                xml_group.attrib["image"] = "AirframeStandardPlane.png"
            for param in group.GetParams():
                if (last_param_name == param.GetName() and not board_specific_param_set) or last_param_name != param.GetName():
                    xml_param = ET.SubElement(xml_group, "airframe")
                    xml_param.attrib["name"] = param.GetName()
                    xml_param.attrib["id"] = param.GetId()
                    xml_param.attrib["maintainer"] = param.GetMaintainer()
                    last_param_name = param.GetName()
                    for code in param.GetFieldCodes():
                        value = param.GetFieldValue(code)
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
                    for code in param.GetOutputCodes():
                        value = param.GetOutputValue(code)
                        xml_field = ET.SubElement(xml_param, "output")
                        xml_field.attrib["name"] = code
                        xml_field.text = value
                if last_param_name != param.GetName():
                    board_specific_param_set = False
        indent(xml_parameters)
        self.xml_document = ET.ElementTree(xml_parameters)

    def Save(self, filename):
        self.xml_document.write(filename, encoding="UTF-8")
