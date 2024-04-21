#!/usr/bin/python3

"""
Migrate c parameter definitions to yaml module definitions.

This script is used to transition legacy c parameter definitions to
yaml module definitions. It parses each specified legacy c file
and produces a corresponding yaml definition in the same directory.
For modules with multiple parameter c files and/or existing module.yaml files,
the resulting list of yaml files must be merged separately, either manually
or with a tool like yq4:

    yq eval-all '. as $item ireduce ({}; . *+ $item)' *.yaml

The legacy files and temporary yaml files can then be manually deleted
and the CMakeLists.txt updated after proofreading.
"""

import argparse
import ast
import sys
import re
import math
import logging
import os
from dataclasses import dataclass, field
from typing import Any

import yaml

global default_var
default_var = {}

@dataclass
class Parameter:
    """
    Single parameter
    """

    name: str
    type: str
    fields: dict[str, str] = field(init=False, default_factory=dict)
    default: str = ""
    category: str = ""
    enum: dict[str, str] = field(init=False, default_factory=dict)
    bitmask: dict[int, str] = field(init=False, default_factory=dict)
    volatile: bool = False
    boolean: bool = False

    def __lt__(self, other):
        return self.name < other.name

@dataclass
class ParameterGroup:
    """
    Single parameter group
    """

    name: str
    parameters: list[Parameter] = field(init=False, default_factory=list)
    no_code_generation: bool = False


class SourceParser:
    """
    Parses provided data and stores all found parameters internally.
    """

    re_split_lines = re.compile(r'[\r\n]+')
    re_comment_start = re.compile(r'^\/\*\*')
    re_comment_content = re.compile(r'^\*\s*(.*)')
    re_comment_tag = re.compile(r'@([a-zA-Z][a-zA-Z0-9_]*)\s*(.*)')
    re_comment_end = re.compile(r'(.*?)\s*\*\/')
    re_parameter_definition = re.compile(r'PARAM_DEFINE_([A-Z_][A-Z0-9_]*)\s*\(([A-Z_][A-Z0-9_]*)\s*,\s*([^ ,\)]+)\s*\)\s*;')
    re_px4_parameter_definition = re.compile(r'PX4_PARAM_DEFINE_([A-Z_][A-Z0-9_]*)\s*\(([A-Z_][A-Z0-9_]*)\s*\)\s*;')
    re_px4_param_default_definition = re.compile(r'#define\s*PARAM_([A-Z_][A-Z0-9_]*)\s*([^ ,\)]+)\s*')
    re_cut_type_specifier = re.compile(r'[a-z]+$')
    re_is_a_number = re.compile(r'^-?[0-9\.]')
    re_remove_dots = re.compile(r'\.+$')
    re_remove_carriage_return = re.compile('\n+')

    valid_tags = set(["group", "board", "min", "max", "unit", "decimal",
                      "increment", "reboot_required", "value", "boolean",
                      "bit", "category", "volatile"])

    # Order of parameter groups
    priority = {
        # All other groups = 0 (sort alphabetically)
        "Miscellaneous": -10
    }

    def __init__(self):
        self.param_groups = {}

    def parse(self, contents):
        """
        Incrementally parse program contents and append all found parameters
        to the list.
        """
        # This code is essentially a comment-parsing grammar. "state"
        # represents parser state. It contains human-readable state
        # names.
        state = None
        for line in self.re_split_lines.split(contents):
            line = line.strip()
            # Ignore empty lines
            if line == "":
                continue
            if self.re_comment_start.match(line):
                state = "wait-short"
                short_desc = None
                long_desc = None
                tags = {}
                def_values = {}
                def_bitmask = {}
            elif state is not None and state != "comment-processed":
                m = self.re_comment_end.search(line)
                if m:
                    line = m.group(1)
                    last_comment_line = True
                else:
                    last_comment_line = False
                m = self.re_comment_content.match(line)
                if m:
                    comment_content = m.group(1)
                    if comment_content == "":
                        # When short comment ends with empty comment line,
                        # start waiting for the next part - long comment.
                        if state == "wait-short-end":
                            state = "wait-long"
                    else:
                        m = self.re_comment_tag.match(comment_content)
                        if m:
                            tag, desc = m.group(1, 2)
                            if (tag == "value"):
                                # Take the meta info string and split
                                # the code and description
                                metainfo = desc.split(" ",  1)
                                def_values[metainfo[0]] = metainfo[1]
                            elif (tag == "bit"):
                                # Take the meta info string and split
                                # the code and description
                                metainfo = desc.split(" ",  1)
                                def_bitmask[metainfo[0]] = metainfo[1]
                            else:
                                tags[tag] = desc
                            current_tag = tag
                            state = "wait-tag-end"
                        elif state == "wait-short":
                            # Store first line of the short description
                            short_desc = comment_content
                            state = "wait-short-end"
                        elif state == "wait-short-end":
                            # Append comment line to the short description
                            short_desc += "\n" + comment_content
                        elif state == "wait-long":
                            # Store first line of the long description
                            long_desc = comment_content
                            state = "wait-long-end"
                        elif state == "wait-long-end":
                            # Append comment line to the long description
                            long_desc += "\n" + comment_content
                        elif state == "wait-tag-end":
                            # Append comment line to the tag text
                            tags[current_tag] += "\n" + comment_content
                        else:
                            raise AssertionError(
                                    "Invalid parser state: %s" % state)
                elif not last_comment_line:
                    # Invalid comment line (inside comment, but not
                    # startin with "*" or "*/". Reset parsed content.
                    state = None
                if last_comment_line:
                    state = "comment-processed"
            else:
                tp = None
                name = None
                defval = ""
                # Non-empty line outside the comment
                m = self.re_px4_param_default_definition.match(line)
                # Default value handling
                if m:
                    name_m, defval_m = m.group(1, 2)
                    default_var[name_m] = defval_m
                m = self.re_parameter_definition.match(line)
                if m:
                    tp, name, defval = m.group(1, 2, 3)
                else:
                    m = self.re_px4_parameter_definition.match(line)
                    if m:
                        tp, name = m.group(1, 2)
                        if (name+'_DEFAULT') in default_var:
                            defval = default_var[name+'_DEFAULT']
                if tp is not None:
                    # Remove trailing type specifier from numbers: 0.1f => 0.1
                    if defval != "" and self.re_is_a_number.match(defval):
                        defval = self.re_cut_type_specifier.sub('', defval)
                    param = Parameter(name=name, type=tp, default=defval)
                    param.fields["short_desc"] = name
                    # If comment was found before the parameter declaration,
                    # inject its data into the newly created parameter.
                    group = "Miscellaneous"
                    if state == "comment-processed":
                        if short_desc is not None:
                            if '\n' in short_desc:
                                raise Exception('short description must be a single line (parameter: {:})'.format(name))
                            if len(short_desc) > 150:
                                raise Exception('short description too long (150 max, is {:}, parameter: {:})'.format(len(short_desc), name))
                            param.fields["short_desc"] = self.re_remove_dots.sub('', short_desc)
                        if long_desc is not None:
                            long_desc = self.re_remove_carriage_return.sub(' ', long_desc)
                            param.fields["long_desc"] = long_desc
                        for tag in tags:
                            if tag == "group":
                                group = tags[tag]
                            elif tag == "volatile":
                                param.volatile = True
                            elif tag == "category":
                                param.category = tags[tag]
                            elif tag == "boolean":
                                param.boolean = True
                            elif tag not in self.valid_tags:
                                sys.stderr.write("Skipping invalid documentation tag: '%s'\n" % tag)
                                return False
                            else:
                                param.fields[tag] = tags[tag]
                        for def_value in def_values:
                            param.enum[def_value] = def_values[def_value]
                        for def_bit in def_bitmask:
                            param.bitmask[def_bit] = def_bitmask[def_bit]
                    # Store the parameter
                    if group not in self.param_groups:
                        self.param_groups[group] = ParameterGroup(group)
                    self.param_groups[group].parameters.append(param)
                state = None
        return True

    def validate(self):
        """
        Validates the parameter meta data.
        """
        seenParamNames = []

        for group in self.parameter_groups:
            for param in sorted(group.parameters):
                name  = param.name
                if len(name) > 16:
                    sys.stderr.write("Parameter Name {0} is too long (Limit is 16)\n".format(name))
                    return False
                board = param.fields.get("board", "")
                # Check for duplicates
                name_plus_board = name + "+" + board
                for seenParamName in seenParamNames:
                    if seenParamName == name_plus_board:
                        sys.stderr.write("Duplicate parameter definition: {0}\n".format(name_plus_board))
                        return False
                seenParamNames.append(name_plus_board)
                # Validate values
                default = param.default
                min = param.fields.get("min", "")
                max = param.fields.get("max", "")
                units = param.fields.get("unit", "")
                if default != "" and not self.is_number(default):
                    sys.stderr.write("Default value not number: {0} {1}\n".format(name, default))
                    return False
                if min != "":
                    if not self.is_number(default):
                        sys.stderr.write("Min value not number: {0} {1}\n".format(name, min))
                        return False
                    if default != "" and float(default) < float(min):
                        sys.stderr.write("Default value is smaller than min: {0} default:{1} min:{2}\n".format(name, default, min))
                        return False
                if max != "":
                    if not self.is_number(max):
                        sys.stderr.write("Max value not number: {0} {1}\n".format(name, max))
                        return False
                    if default != "" and float(default) > float(max):
                        sys.stderr.write("Default value is larger than max: {0} default:{1} max:{2}\n".format(name, default, max))
                        return False
                for code in sorted(param.enum, key=float):
                    if not self.is_number(code):
                        sys.stderr.write("Min value not number: {0} {1}\n".format(name, code))
                        return False
                    if param.enum.get(code, "") == "":
                        sys.stderr.write("Description for enum value is empty: {0} {1}\n".format(name, code))
                        return False
                for index in sorted(param.bitmask.keys(), key=float):
                    if not self.is_number(index):
                        sys.stderr.write("bit value not number: {0} {1}\n".format(name, index))
                        return False
                    if not int(min) <= math.pow(2, int(index)) <= int(max):
                        sys.stderr.write("Bitmask bit must be between {0} and {1}: {2} {3}\n".format(min, max, name, math.pow(2, int(index))))
                        return False
                    if param.bitmask.get(index, "") == "":
                        sys.stderr.write("Description for bitmask bit is empty: {0} {1}\n".format(name, index))
                        return False
        return True

    def is_number(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False

    @property
    def parameter_groups(self) -> list[ParameterGroup]:
        """
        Returns the parsed list of parameters. Every parameter is a Parameter
        object. Note that returned object is not a copy. Modifications affect
        state of the parser.
        """
        groups = self.param_groups.values()
        groups = sorted(groups, key=lambda x: x.name)
        groups = sorted(groups, key=lambda x: self.priority.get(x.name, 0),
                        reverse=True)

        return groups


def parse(filename: str) -> list[ParameterGroup]:
    with open(filename) as f:
        cdata = f.read()

    parser = SourceParser()
    if not parser.parse(cdata):
        logging.error(f"Error parsing c parameter file {filename}")
        sys.exit(1)
    if not parser.validate():
        logging.error(f"Error while validating c parameter file {filename}")
        sys.exit(1)

    return parser.parameter_groups


def cast(val: str) -> Any:
    if val == "true":
        return True
    elif val == "false":
        return False
    try:
        return ast.literal_eval(val)
    except ValueError:
        return val


def generate_yaml(filename: str, groups: list[ParameterGroup]) -> str:
    data = dict()

    module_name = os.path.dirname(os.path.realpath(filename)).split(os.sep)[-1]

    data["module_name"] = module_name
    data["parameters"] = list()

    for group in groups:
        g = dict()
        g["group"] = group.name
        g["definitions"] = dict()

        for parameter in group.parameters:
            p = dict()

            p["description"] = dict()
            p["description"]["short"] = parameter.fields["short_desc"]
            del parameter.fields["short_desc"]
            if "long_desc" in parameter.fields:
                p["description"]["long"] = parameter.fields["long_desc"]
                del parameter.fields["long_desc"]

            if parameter.category != "":
                p["category"] = parameter.category.title()
            # the enum check has to happen first
            # since some parameters are both boolean and enum at the same time
            # (even with more than 0 and 1 as options for some reason) so let's assume
            # they are enums not booleans
            if len(parameter.enum) > 0:
                p["type"] = "enum"
                p["values"] = dict()
                for key, val in parameter.enum.items():
                    try:
                        p["values"][int(key)] = val
                    except ValueError:
                        p["values"][float(key)] = val
                        p["type"] = "float"
            elif parameter.boolean:
                p["type"] = "boolean"
            elif len(parameter.bitmask) > 0:
                p["type"] = "bitmask"
                p["bit"] = dict()
                for key, val in parameter.bitmask.items():
                    p["bit"][int(key)] = val.strip()
            elif parameter.type == "FLOAT":
                p["type"] = "float"
            else:
                p["type"] = "int32"
            p["default"] = cast(parameter.default)

            if parameter.volatile:
                p["volatile"] = bool(parameter.volatile)

            for key, val in parameter.fields.items():
                try:
                    p[key] = cast(val)
                except SyntaxError:
                    p[key] = val

            g["definitions"][parameter.name] = p
        data["parameters"].append(g)

    return yaml.dump(data, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(description="Migrate legacy c parameter definitions to yaml")
    parser.add_argument('input_file', nargs='+', help='input file(s)')
    parser.add_argument('--update-cmake', action=argparse.BooleanOptionalAction)
    args = parser.parse_args()

    input_files = args.input_file
    update_cmake = args.update_cmake

    logging.basicConfig(level=logging.INFO)

    for filename in input_files:
        logging.info(f"Migrating c parameter file {filename}")

        parameter_groups = parse(filename)
        output = generate_yaml(filename, parameter_groups)

        dirname, fname = os.path.split(os.path.realpath(filename))
        fname = fname.split(".")[0]
        with open(os.path.join(dirname, f"module_{fname}.yaml"), "w") as f:
            f.write(output)
        if update_cmake:
            with open(os.path.join(dirname, "CMakeLists.txt"), "r+") as f:
                content = f.read()
                if "MODULE_CONFIG" not in content:
                    try:
                        index = content.index("DEPENDS")
                    except ValueError:
                        index = content.index("px4_add_module")
                        index += content[index:].index(")")
                    content = content[:index] + "MODULE_CONFIG\n\t\tmodule.yaml\n\t" + content[index:]
                    f.seek(0)
                    f.write(content)
                    f.truncate()


if __name__ == "__main__":
    main()
