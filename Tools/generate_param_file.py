import argparse, sys, struct, os
import xml.etree.ElementTree as ET


def getOptions(args=sys.argv[1:]):
    parser = argparse.ArgumentParser(description="Parses command.")
    parser.add_argument("-i", "--input", help="Input file.")
    parser.add_argument("-o", "--output", help="Output file.")
    parser.add_argument("-p", "--param_xml", help="Parameter xml file.")
    parser.add_argument("-g", "--gen_romfs_dir", help="Generated ROMFS root directory.")
    options = parser.parse_args(args)
    return options

options = getOptions(sys.argv[1:])

f_input = open(options.input, "r")

# get the filename and figure out if we should create a parameter file or not
# try to find an autostart ID in the filename and if it exists, then check if the
# corresponding airframe config file exists in the generated ROMFS directly. If it does not exist
# then don't create the corresponding parameter file.
basename = os.path.basename(options.input)

try:
    autostart_id_str = basename.split(".")[0]
    autostart_id = int(autostart_id_str)

    found_airframe_file = False
    # this parameter file belongs to a specific airframe, check if this builds has it included
    for file_name in os.listdir(options.gen_romfs_dir + "/init.d/airframes"):
        if file_name.startswith(autostart_id_str):
            found_airframe_file = True
            break

    if found_airframe_file == False:
        f_input.close()
        f_input = None
except:
    pass


if f_input is not None:
    f_output = open(options.output, "w+b")
    param_xml_file = options.param_xml

    xml_tree = ET.parse(param_xml_file)
    xml_root = xml_tree.getroot()

    # write some kind of header
    f_output.write(struct.pack('I', 0))

    for line in f_input.readlines():
        if not line.strip() or line.strip().startswith("#"):
            continue

        line_split = line.rstrip("\n").split("#")[0].split(" ")

        param_name = ""
        param_value_str = ""

        for item in line_split:
            if len(item) > 0 and len(param_name) == 0:
                param_name = item.strip()
            elif len(item) > 0 and len(param_value_str) == 0:
                param_value_str = item.strip()

        param_element = xml_root.find('.//parameter[@name="%s"]' % (param_name))

        if param_element is None:
            continue

        param_type_str = param_element.attrib["type"]
        param_value_bytes = 0
        param_type = 0

        if param_type_str == "INT32":
            param_value_bytes = struct.pack("i", int(float(param_value_str)))
            param_type = 16
        elif param_type_str == "FLOAT":
            param_type = 1
            param_value_bytes = struct.pack("d", float(param_value_str))
        else:
            print("unknown param type")
            exit(1)

        f_output.write(struct.pack('B', param_type))
        f_output.write(bytes(param_name, 'ascii') + struct.pack('B', 0))
        f_output.write(param_value_bytes)

    f_output.write(struct.pack('B', 0))
    f_input.close()
    f_output.close()
