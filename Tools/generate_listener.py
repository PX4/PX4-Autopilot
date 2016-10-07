#!/usr/bin/python


from __future__ import print_function
import argparse
import glob
import os
import re
from string import Template

"""Generate the listener systemcmd.

This script generates the C++ code which is behind the listener command
which allows for easier debugging by printing out uORB messages.

Usage: generate_listener src_dir output_file
"""

# Ignore the biggest topics on NuttX to save flash
ignore_on_nuttx_list = [
    "hil_sensor",
    "ekf2_replay"
]


def main():
    """Parse the msg files and write the listener using the template."""

    # Parse and verify user inputs.
    args = parse_args()

    # Get the paths of all msg files.
    msg_paths = glob.glob(args.src_dir + "/msg/*.msg")

    # Parse the message files and put all data in a dict.
    msg_dict = get_msg_data(msg_paths)

    # Read in the template file.
    with open(args.src_dir +
              "/src/systemcmds/topic_listener/topic_listener.cpp.in", "r") as f:
        template = Template(f.read())

    # Assemble the needed substitutions.
    mapping = {'include_topics': assemble_includes(msg_dict),
               'topic_classes': assemble_classes(msg_dict),
               'factory': assemble_factory(msg_dict)}

    # Use the template and substitude the ${placeholders}.
    substituted = template.substitute(mapping)

    # (Over)write the output file.
    with open(args.output_file, "w") as f:
        f.write(substituted)


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description="Generate listener module")
    parser.add_argument("src_dir",
                        help="The top level src directory of Firmware.")
    parser.add_argument("output_file",
                        help="The generated output file.")
    return parser.parse_args()


def get_msg_data(msg_paths):
    """Read in all files and store data in dict."""

    msg_dict = dict()
    for msg_path in msg_paths:
        elements = parse_msg(msg_path)
        if elements is None:
            # In case some file is rather empty.
            continue

        # Use the msg_name as the key.
        msg_name = get_msg_name(msg_path)
        msg_dict[msg_name] = elements

    return msg_dict


def parse_msg(file_path):
    """Parse one msg file."""
    elements = []
    with open(file_path, 'r') as f:
        for line in f.readlines():
            items = line.strip().split()
            if not items:
                continue

            element = parse_msg_items(items)
            if element:
                elements.append(element)

    return elements


map_from_msg_types_to_cpp_types = {
    'float32': 'float',
    'float64': 'double',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'bool': 'bool'
}


class Element(object):
    """One element represents one type line in a msg file."""
    def __init__(self):
        self.array_len = None
        self.type_name = ""
        self.var_name = ""


def parse_msg_items(items):
    """Try to make sense from the items of a line."""
    new_element = Element()

    # First try to find lines/entries that are arrays:
    #
    # This matches agains word123[42] and extracts
    # word123 as type and 42 as array_len.
    # \w* -> word123
    # \[\d*\] -> [42]
    # (?P<name>REGEX) assigns it to group('name')
    m = re.match(r"(?P<type>\w*)\[(?P<array_len>\d*)\]", items[0])
    if m:
        type_name = m.group('type')
        new_element.array_len = m.group('array_len')
        new_element.var_name = items[1]
    else:
        # There needs to be at least type and variable name.
        if len(items) < 2:
            return None

        # We want to filter out constants such as "uint8 CONSTANT=5"
        if "=" in items[1]:
            return None

        # We also want to filter out constants such as "uint8 CONSTANT = 5"
        if len(items) > 2 and "=" in items[2]:
            return None

        # Let's assume it's valid now and not an array
        type_name = items[0]
        new_element.var_name = items[1]

    # Check if we can make use of the type actually:
    if type_name not in map_from_msg_types_to_cpp_types:
        # Unknown type or other line, let's throw it away.
        return None

    # Let's use the C++ type name from now on
    new_element.type_name = map_from_msg_types_to_cpp_types[type_name]
    return new_element


def get_msg_name(file_path):
    """Basically get the filename without ending."""
    (head, tail) = os.path.split(file_path)
    msg_name = tail.split('.')[0]
    return msg_name


function_around_variable = {
    'float': "(double){}",
    'double': "(double){}",
    'uint64_t': "{}",
    'int64_t': "{}",
    'uint32_t': "{}",
    'int32_t': "{}",
    'uint16_t': "(unsigned){}",
    'int16_t': "(int){}",
    'uint8_t': "(unsigned){}",
    'int8_t': "(int){}",
    'bool': "({}) ? \"True\" : \"False\""
}

type_specifier_for_cpp_type = {
    'float': "%8.4f",
    'double': "%8.4",
    'uint64_t': "%\" PRIu64 \"",
    'int64_t': "%\" PRId64 \"",
    'uint32_t': "%u",
    'int32_t': "%d",
    'uint16_t': "%u",
    'int16_t': "%d",
    'uint8_t': "%u",
    'int8_t': "%d",
    'bool': "%s"
}


def get_print_function(element):
    """Construct print function of an element."""
    if element.array_len is not None:
        return get_print_function_for_array(element)
    else:
        return get_print_function_for_single(element)


def get_print_function_for_array(element):
    """Construct print function of an array element."""

    # The variable name needs to be constructed first because
    # it will get casts around it.
    variable = "_container.{}[j]".format(element.var_name)

    # Construct the ugly loop with all it needs.
    text = "\t\tPX4_INFO_RAW(\"{}: \");\n".format(element.var_name)
    text = ""
    text += "\t\tfor (int j = 0; j < {}; ++j) {{\n".format(element.array_len)
    text += "\t\t\tPX4_INFO_RAW(\"{} \", {});\n".format(
        type_specifier_for_cpp_type[element.type_name],
        function_around_variable[element.type_name].format(variable))
    text += "\t\t}\n"
    text += "\t\tPX4_INFO_RAW(\"\\n\");\n"
    return text


def get_print_function_for_single(element):
    """Construct print function of single/non-array element."""
    # We need the variable first again.
    variable = "_container.{}".format(element.var_name)

    # Now just print it in one line.
    text = "\t\tPX4_INFO_RAW(\"{}: {}\\n\", {});\n".format(
        element.var_name,
        type_specifier_for_cpp_type[element.type_name],
        function_around_variable[element.type_name].format(variable))

    return text


def assemble_includes(msg_dict):
    """Get the text to include all topic files."""
    text = ""
    for msg in msg_dict.keys():
        text += "#include <uORB/topics/{}.h>\n".format(msg)
    return text


def assemble_classes(msg_dict):
    """Assemble the class which inherit from the Topic class."""
    text = ""

    # Go through all msgs and their elements.
    for msg, elements in msg_dict.items():

        if msg in ignore_on_nuttx_list:
            text += "#ifndef __PX4_NUTTX\n"

        text += "class Topic_{} : public Topic {{\n".format(msg)
        text += "public:\n"
        text += "\tTopic_{}(orb_id_t id)\n".format(msg)
        text += "\t\t: Topic(id),\n"
        text += "\t_container{}\n"
        text += "\t{\n"
        text += "\t\t_data = &_container;\n"
        text += "\t}\n"

        print_function = ""
        print_function += "\t\tPX4_INFO_RAW(\"timestamp: %\" PRIu64 \"\\n\"," \
                          "_container.timestamp);\n"

        for element in elements:
            print_function += get_print_function(element)

        print_function += "\t\tPX4_INFO_RAW(\"\\n\");\n"

        text += "\tvoid print_specific() override\n"
        text += "\t{\n"
        text += print_function
        text += "\t}\n"

        # text += "\tbool update() override"
        # text += "\t{\n"
        # text += "\t\tbool updated = false;\n"
        # text += "\t\torb_check(_sub, &updated);\n"
        # text += "\t\tif (updated) {\n"
        # text += "\t\t\torb_copy(_id, _sub, (void *)&_container);\n"
        # text += "\t\t\treturn true;\n"
        # text += "\t\t} else {\n"
        # text += "\t\t\treturn false;\n"
        # text += "\t\t}\n"
        # text += "\t}\n"

        text += "private:\n"
        text += "\tstruct {}_s _container;\n".format(msg)
        text += "};\n\n"

        if msg in ignore_on_nuttx_list:
            text += "#endif\n"

    return text


def assemble_factory(msg_dict):
    """Put together the factory which creates the objects."""
    text = ""

    for index, msg in enumerate(msg_dict.keys()):

        # The first time is an "if", after that it's "else if"
        if index > 0:
            text += "\telse "
        else:
            text += "\t"

        text += "if (strncmp(name, \"{}\", 50) == 0) {{\n".format(msg)

        if msg in ignore_on_nuttx_list:
            text += "#ifdef __PX4_NUTTX\n"
            text += "\t\tPX4_WARN(\"Topic not available on NuttX\");\n"
            text += "#else\n"

        text += "\t\ttopic = new Topic_{}(ORB_ID({}));\n".format(msg, msg)

        if msg in ignore_on_nuttx_list:
            text += "#endif\n"

        text += "\t}\n"

    return text


if __name__ == '__main__':
    main()
