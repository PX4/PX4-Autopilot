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
        xml_version = ET.SubElement(xml_parameters, "airframe_version_major")
        xml_version.text = "1"
        xml_version = ET.SubElement(xml_parameters, "airframe_version_minor")
        xml_version.text = "1"
        last_param_name = ""
        board_specific_param_set = False
        for group in groups:
            xml_group = ET.SubElement(xml_parameters, "airframe_group")
            xml_group.attrib["name"] = group.GetName()
            if (group.GetName() == "Standard Plane"):
                xml_group.attrib["image"] = "Plane"
            elif (group.GetName() == "Flying Wing"):
                xml_group.attrib["image"] = "FlyingWing"
            elif (group.GetName() == "Quadrotor x"):
                xml_group.attrib["image"] = "QuadRotorX"
            elif (group.GetName() == "Quadrotor +"):
                xml_group.attrib["image"] = "QuadRotorPlus"
            elif (group.GetName() == "Hexarotor x"):
                xml_group.attrib["image"] = "HexaRotorX"
            elif (group.GetName() == "Hexarotor +"):
                xml_group.attrib["image"] = "HexaRotorPlus"
            elif (group.GetName() == "Octorotor +"):
                xml_group.attrib["image"] = "OctoRotorPlus"
            elif (group.GetName() == "Octorotor x"):
                xml_group.attrib["image"] = "OctoRotorX"
            elif (group.GetName() == "Octorotor Coaxial"):
                xml_group.attrib["image"] = "OctoRotorXCoaxial"
            elif (group.GetName() == "Octo Coax Wide"):
                xml_group.attrib["image"] = "OctoRotorXCoaxial"
            elif (group.GetName() == "Quadrotor Wide"):
                xml_group.attrib["image"] = "QuadRotorWide"
            elif (group.GetName() == "Quadrotor H"):
                xml_group.attrib["image"] = "QuadRotorH"
            elif (group.GetName() == "Simulation"):
                xml_group.attrib["image"] = "AirframeSimulation"
            elif (group.GetName() == "Plane A-Tail"):
                xml_group.attrib["image"] = "PlaneATail"
            elif (group.GetName() == "VTOL Duo Tailsitter"):
                xml_group.attrib["image"] = "VTOLDuoRotorTailSitter"
            elif (group.GetName() == "Standard VTOL"):
                xml_group.attrib["image"] = "VTOLPlane"
            elif (group.GetName() == "VTOL Quad Tailsitter"):
                xml_group.attrib["image"] = "VTOLQuadRotorTailSitter"
            elif (group.GetName() == "VTOL Tiltrotor"):
                xml_group.attrib["image"] = "VTOLTiltRotor"
            elif (group.GetName() == "Coaxial Helicopter"):
                xml_group.attrib["image"] = "HelicopterCoaxial"
            elif (group.GetName() == "Hexarotor Coaxial"):
                xml_group.attrib["image"] = "Y6A"
            elif (group.GetName() == "Y6B"):
                xml_group.attrib["image"] = "Y6B"
            elif (group.GetName() == "Tricopter Y-"):
                xml_group.attrib["image"] = "YMinus"
            elif (group.GetName() == "Tricopter Y+"):
                xml_group.attrib["image"] = "YPlus"
            elif (group.GetName() == "Rover"):
                xml_group.attrib["image"] = "Rover"
            elif (group.GetName() == "Boat"):
                xml_group.attrib["image"] = "Boat"
            else:
                xml_group.attrib["image"] = "AirframeUnknown"
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
                        valstrs = value.split(";")
                        xml_field = ET.SubElement(xml_param, "output")
                        xml_field.attrib["name"] = code
                        for attrib in valstrs[1:]:
                            attribstrs = attrib.split(":")
                            xml_field.attrib[attribstrs[0].strip()] = attribstrs[1].strip()
                        xml_field.text = valstrs[0]
                if last_param_name != param.GetName():
                    board_specific_param_set = False
        indent(xml_parameters)
        self.xml_document = ET.ElementTree(xml_parameters)

    def Save(self, filename):
        self.xml_document.write(filename, encoding="UTF-8")
