import xml.etree.ElementTree as ET
import codecs
import os

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

    def __init__(self, groups, board, inject_xml_file_name):
        unique_strings = set()
        xml_parameters = ET.Element("properties")

        for group in groups:
            group_name=group.GetName()
            if group_name not in unique_strings:
                xml_entry = ET.SubElement(xml_parameters, "entry")
                xml_entry.attrib["key"] = group_name
                #xml_entry.attrib["type"] = 'group'
                xml_entry.text = group_name
                unique_strings.add(group_name)
            else:
                pass
            
            for param in group.GetParams():
                param_name = param.GetName()
                param_default = param.GetDefault()
                param_type = param.GetType()
                param_volatile = param.GetVolatile()
                param_category = param.GetCategory()
                #print("DEBUG: param_name: %s" % param_name)
                #print("DEBUG: param_default: %s" % param_default)
                #print("DEBUG: param type: %s" % param_type)
                #print("DEBUG: param_volatile: %s" % param_volatile)
                #print("DEBUG: param_category: %s" % param_category)

                #Add param_category (if exists)
                if param_category not in unique_strings and param_category:
                    xml_entry = ET.SubElement(xml_parameters, "entry")
                    xml_entry.attrib["key"] = param_category
                    #xml_entry.attrib["type"] = 'param_category'
                    #xml_entry.attrib["param"] = 'param_name'
                    xml_entry.text = param_category
                    unique_strings.add(param_category)
                else:
                    pass
                
                for code in param.GetFieldCodes():
                    value = param.GetFieldValue(code)
                    #print('DEBUG: code: %s, value: %s' % (code, value))
                    if code in ['short_desc','long_desc']:  #Other fields are not translateable
                        if value not in unique_strings and value:
                            xml_entry = ET.SubElement(xml_parameters, "entry")
                            xml_entry.attrib["key"] = param_name+'_'+code
                            #xml_entry.attrib["type"] = code
                            #xml_entry.attrib["param"] = param_name
                            xml_entry.text = value
                            unique_strings.add(value)
                        else:
                            pass             



                if len(param.GetEnumCodes()) > 0:
                    for code in param.GetEnumCodes():
                        value = param.GetEnumValue(code)
                        #print('DEBUG: code: %s, value: %s' % (code, value))
                        if value not in unique_strings and value:
                            xml_entry = ET.SubElement(xml_parameters, "entry")
                            xml_entry.attrib["key"] = '%s_enumval_%s' % (param_name,code)
                            #xml_entry.attrib["type"] = 'enumvalue'
                            #xml_entry.attrib["value"] = code
                            #xml_entry.attrib["param"] = param_name
                            xml_entry.text = value
                            unique_strings.add(value)
                        else:
                            pass  

                if len(param.GetBitmaskList()) > 0:
                    for index in param.GetBitmaskList():
                        value = param.GetBitmaskBit(index)
                        #print('DEBUG: index: %s, value: %s' % (index, value))
                        if value not in unique_strings and value:
                            xml_entry = ET.SubElement(xml_parameters, "entry")
                            xml_entry.attrib["key"] = '%s_bit_%s' % (param_name,index)
                            #xml_entry.attrib["type"] = 'bit'
                            #xml_entry.attrib["value"] = index
                            #xml_entry.attrib["param"] = param_name
                            xml_entry.text = value
                            unique_strings.add(value)
                        else:
                            pass  



        indent(xml_parameters)
        self.xml_document = ET.ElementTree(xml_parameters)

    def Save(self, filename):
        # Create target directory & all intermediate directories if don't exists
        dirname=filename.rsplit('/',1)[0]
        if not os.path.exists(dirname):
            os.makedirs(dirname)

        self.xml_document.write(filename, encoding="UTF-8")
        # Add doctype (Clunky, but not possible to avoid in ElementTree)
        f=open(filename, "r")
        filetext=f.read()
        f.close
        filetext=filetext.replace('<properties>','<!DOCTYPE properties SYSTEM "[http://java.sun.com/dtd/properties.dtd](http://java.sun.com/dtd/properties.dtd)">\n<properties>')
        with open(filename, "w") as f:
            f.write(filetext)

