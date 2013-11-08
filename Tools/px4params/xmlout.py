import output
from xml.dom.minidom import getDOMImplementation

class XMLOutput(output.Output):
    def Generate(self, groups):
        impl = getDOMImplementation()
        xml_document = impl.createDocument(None, "parameters", None)
        xml_parameters = xml_document.documentElement
        for group in groups:
            xml_group = xml_document.createElement("group")
            xml_group.setAttribute("name", group.GetName())
            xml_parameters.appendChild(xml_group)
            for param in group.GetParams():
                xml_param = xml_document.createElement("parameter")
                xml_group.appendChild(xml_param)
                for code in param.GetFieldCodes():
                    value = param.GetFieldValue(code)
                    xml_field = xml_document.createElement(code)
                    xml_param.appendChild(xml_field)
                    xml_value = xml_document.createTextNode(value)
                    xml_field.appendChild(xml_value)
        return xml_document.toprettyxml(indent="    ", newl="\n", encoding="utf-8")
