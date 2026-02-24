#!/usr/bin/env python3

"""
Generate docs from .msg files
Also generates docs/en/middleware/dds_topics.md from dds_topics.yaml
"""

import os
import argparse
import sys
import re

VALID_FIELDS = { #Note, also have to add the message types as those can be fields
    'uint64',
    'uint16',
    'uint8',
    'uint32'
}

ALLOWED_UNITS = set(["m", "m/s", "m/s^2", "(m/s)^2", "deg", "deg/s", "rad", "rad/s", "rad^2", "rpm" ,"V", "A", "mA", "mAh", "W", "dBm", "h", "s", "ms", "us", "Ohm", "MB", "Kb/s", "degC","Pa","%","-"])
invalid_units = set()
ALLOWED_FRAMES = set(["NED","Body"])
ALLOWED_INVALID_VALUES = set(["NaN", "0"])
ALLOWED_CONSTANTS_NOT_IN_ENUM = set(["ORB_QUEUE_LENGTH","MESSAGE_VERSION"])

class Error:
    def __init__(self, type, message, linenumber=None, issueString = None, field = None):
        self.type = type
        self.message = message
        self.linenumber = linenumber
        self.issueString = issueString
        self.field = field

    def display_error(self):
        #print(f"Debug: Error: display_error")


        if 'trailing_whitespace' == self.type:
            if self.issueString.strip():
                print(f"NOTE: Line has trailing whitespace ({self.message}: {self.linenumber}): {self.issueString}")
            else:
                print(f"NOTE: Line has trailing whitespace ({self.message}: {self.linenumber})")
        elif 'leading_whitespace_field_or_constant' == self.type:
            print(f"NOTE: Whitespace before field or constant ({self.message}: {self.linenumber}): {self.issueString}")
        elif 'field_or_constant_has_multiple_whitepsace' == self.type:
            print(f"NOTE: Field/constant has more than one sequential whitespace character ({self.message}: {self.linenumber}): {self.issueString}")
        elif 'empty_start_line' == self.type:
            print(f"NOTE: Empty line at start of file ({self.message}: {self.linenumber})")
        elif 'internal_comment' == self.type:
            print(f"NOTE: Internal Comment ({self.message}: {self.linenumber})\n {self.issueString}")
        elif 'internal_comment_empty' == self.type:
            print(f"NOTE: Empty Internal Comment ({self.message}: {self.linenumber})")
        elif 'summary_missing' == self.type:
            print(f"WARNING: No message description ({self.message})")
        elif 'topic_error' == self.type:
            print(f"NOTE: TOPIC ISSUE: {self.issueString}")
        elif 'unknown_unit' == self.type:
            print(f"WARNING: Unknown Unit: [{self.issueString}] on `{self.field}` ({self.message}: {self.linenumber})")
        elif 'constant_not_in_assigned_enum' == self.type:
            print(f"WARNING: `{self.issueString}` constant: Prefix not in `@enum` field metadata ({self.message}: {self.linenumber})")
        elif 'unknown_invalid_value' == self.type:
            print(f"WARNING: Unknown @invalid value: [{self.issueString}] on `{self.field}` ({self.message}: {self.linenumber})")
        elif 'unknown_frame' == self.type:
            print(f"WARNING: Unknown @frame: [{self.issueString}] on `{self.field}` ({self.message}: {self.linenumber})")
        elif 'command_no_params_pipes' == self.type:
            print(f"WARNING: `{self.field}` command has no parameters (pipes): [{self.issueString}] ({self.message}: {self.linenumber})")
        elif 'command_missing_params' == self.type:
            print(f"WARNING: `{self.field}` command missing params - should be 7 params surrounded by 8 pipes: [{self.issueString}] ({self.message}: {self.linenumber})")
        elif 'command_too_many_params' == self.type:
            print(f"WARNING: `{self.field}` command too many params (should be 7). Extras: [{self.issueString}] ({self.message}: {self.linenumber})")


        else:
            self.display_info()

    def display_info(self):
        """
        Display info about an error.
        Used as a fallback if error does not have specific printout in display_error()
        """
        #print(f"Debug: Error: display_info")
        print(f" type: {self.type}, message: {self.message}, linenumber: {self.linenumber}, issueString: {self.issueString}, field: {self.field}")

class Enum:
    def __init__(self, name, parentMessage):
        self.name = name
        self.parent = parentMessage
        self.enumValues = dict()

    def display_info(self):
        """
        Display info about an enum
        """
        print(f"Debug: Enum: display_info")
        print(f" name: {self.name}")
        for key, value in self.enumValues.items():
            value.display_info()

class ConstantValue:
    def __init__(self, name, type, value, comment, line_number):
        self.name = name.strip()
        self.type = type.strip()
        self.value = value.strip()
        self.comment = comment
        self.line_number = line_number

        if not self.value:
            print(f"Debug WARNING: NO VALUE in ConstantValue: {self.name}")  ## TODO make into ERROR
            exit()

        # TODO if value or name are empty, error

    def display_info(self):
        print(f"Debug: ConstantValue: display_info")
        print(f" name: {self.name}, type: {self.type}, value: {self.value}, comment: {self.comment}, line: {self.line_number}")


class CommandParam:
    """
    Represents an individual param in a command constant
    Encapsulates parsing of the param to extract units etc.
    """

    def __init__(self, num, paramText, line_number, parentCommand):
        self.paramNum = num
        self.paramText = paramText.strip()
        self.enum = None
        self.range = None
        #self.type = type
        self.units = []
        self.enums = []
        self.minValue = None
        self.maxValue = None
        self.invalidValue = None
        self.frameValue = None
        self.lineNumber = line_number
        self.parent = parentCommand
        self.parentMessage = self.parent.parent

        match = None
        if self.paramText:
            match = re.match(r'^((?:\[[^\]]*\]\s*)+)(.*)$', paramText)
        self.description = paramText
        bracketed_part = None
        if match:
            bracketed_part = match.group(1).strip() # .strip() removes trailing whitespace from the bracketed part
            self.description = match.group(2).strip()
        if bracketed_part:
          # get units
            bracket_content_matches = re.findall(r'\[(.*?)\]', bracketed_part)
            #print(f"DEBUG: bracket_content_matches: {bracket_content_matches}")
            for item in bracket_content_matches:
                item = item.strip()
                if item.startswith('@'): # Not a unit:
                    if item.startswith('@enum'):
                        item = item.split(" ")
                        enum = item[1].strip()
                        if enum and enum not in self.enums:
                            self.enums.append(enum)

                        # Create parent enum objects for any enums created in this step
                        for enumName in self.enums:
                            if not enumName in self.parentMessage.enums:
                                self.parentMessage.enums[enumName]=Enum(enumName,self.parentMessage)

                    elif item.startswith('@range'):
                        item = item[6:].strip().split(",")
                        self.range = item
                        self.minValue = item[0].strip()
                        self.maxValue = item[1].strip()
                    elif item.startswith('@invalid'):
                        self.invalidValue = item[8:].strip()
                        #TODO: Do we require a description? (not currently)
                        if self.invalidValue.split(" ")[0] not in ALLOWED_INVALID_VALUES:
                            print(f"TODO: Command param do not support @invalid: {self.invalidValue}")
                            """
                            error = Error("unknown_invalid_value", self.parent.filename, self.lineNumber, self.invalidValue, self.name)
                            #error.display_error()
                            if not "unknown_invalid_value" in self.parent.errors:
                                self.parent.errors["unknown_invalid_value"] = []
                            self.parent.errors["unknown_invalid_value"].append(error)
                            """

                    elif item.startswith('@frame'):
                        self.frameValue = item[6:].strip()
                        print(f"TODO: Command param do not support @frame: {self.frameValue}")
                        """
                        if self.frameValue not in ALLOWED_FRAMES:
                            error = Error("unknown_frame", self.parent.filename, self.lineNumber, self.frameValue, self.name)
                            #error.display_error()
                            if not "unknown_frame" in self.parent.errors:
                                self.parent.errors["unknown_frame"] = []
                            self.parent.errors["unknown_frame"].append(error)
                        """
                    else:
                        print(f"WARNING: Unhandled metadata in message comment: {item}")
                        # TODO - report errors for different kinds of metadata
                        exit()

                else: # bracket is a unit
                    unit = item.strip()

                    if item == "-":
                        unit = ""

                    if unit and unit not in self.units:
                        self.units.append(unit)

                    if unit not in ALLOWED_UNITS:
                        invalid_units.add(unit)
                        error = Error("unknown_unit", self.parentMessage.filename, self.lineNumber, unit, self.parent.name)
                        #error.display_error()
                        if not "unknown_unit" in self.parentMessage.errors:
                            self.parentMessage.errors["unknown_unit"] = []
                        self.parentMessage.errors["unknown_unit"].append(error)


    def display_info(self):
        print(f"Debug: CommandParam: display_info")
        print(f" id: {self.paramNum}")
        print(f"   paramText: {self.paramText}\n  unit:  {self.units}\n  enums: {self.enums}\n  lineNumber: {self.lineNumber}\n  range: {self.range}\n  minValue: {self.minValue}\n  maxValue: {self.maxValue}\n  invalidValue: {self.invalidValue}\n  frameValue: {self.frameValue}\n  parent: {self.parent}\n  ")



class CommandConstant:
    """
    Represents a constant that is a command definition.
    Encapsulates parsing of the command format.
    The individual params are further parsed in CommandParam
    """
    def __init__(self, name, type, value, comment, line_number, parentMessage):
        self.name = name.strip()
        self.type = type.strip()
        self.value = value.strip()
        self.comment = comment
        self.line_number = line_number
        self.parent = parentMessage

        self.description = self.comment
        self.param1 = None
        self.param2 = None
        self.param3 = None
        self.param4 = None
        self.param5 = None
        self.param6 = None
        self.param7 = None

        if not self.value:
            print(f"Debug WARNING: NO VALUE in CommandConstant: {self.name}")  ## TODO make into ERROR
            exit()

        if not self.comment: # This is an bug for a command
            #print(f"Debug WARNING: NO COMMENT in CommandConstant: {self.name}")  ## TODO make into ERROR
            return

        # Parse command comment to get the description and parameters.
        # print(f"Debug CommandConstant: {self.comment}")
        if not "|" in self.comment:
            # This is an error for a command constant
            error = Error("command_no_params_pipes", self.parent.filename, self.line_number, self.comment, self.name)
            #error.display_error()
            if not "command_no_params_pipes" in self.parent.errors:
                self.parent.errors["command_no_params_pipes"] = []
            self.parent.errors["command_no_params_pipes"].append(error)
            return

        # Split on pipes
        commandSplit = self.comment.split("|")
        if len(commandSplit) < 9:
            # Should 7 pipes, so each command is fully surrounded
            error = Error("command_missing_params", self.parent.filename, self.line_number, self.comment, self.name)
            #error.display_error()
            if not "command_missing_params" in self.parent.errors:
                self.parent.errors["command_missing_params"] = []
            self.parent.errors["command_missing_params"].append(error)

        self.description = commandSplit[0].strip()
        self.description = self.description if self.description else None

        params_to_update = commandSplit[1:8]

        for i, value in enumerate(params_to_update, start=1):
            if value.strip():
                # parse the param
                param = CommandParam(i, value, self.line_number, self)
                #param.display_info() # DEBUG CODE XXX
                setattr(self, f"param{i}", param)
                # parse the param

        if len(commandSplit) > 8:
            extras = commandSplit[8:]
            error = Error("command_too_many_params", self.parent.filename, self.line_number, extras, self.name)
            if not "command_too_many_params" in self.parent.errors:
                self.parent.errors["command_too_many_params"] = []
            self.parent.errors["command_too_many_params"].append(error)


        # TODO if value or name are empty, error

    def markdown_out(self):
        #print("DEBUG: CommandConstant.markdown_out")
        output = f"""### {self.name} ({self.value})

{self.description}

Param | Units | Range/Enum | Description
--- | --- | --- | ---
"""
        for i in range(1, 8):
            attr_name = f"param{i}"
            # getattr returns None if the attribute doesn't exist
            val = getattr(self, attr_name, None)

            if val is not None:
                rangeVal = ""
                if val.minValue or val.maxValue:
                    rangeVal = f"[{val.minValue if val.minValue else '-'} : {val.maxValue if val.maxValue else '-' }]"

                output+=f"{i} | {", ".join(val.units)}|{', '.join(f"[{e}](#{e})" for e in val.enums)}{rangeVal} | {val.description}\n"
            else:
                output+=f"{i} | | | ?\n"

        output+=f"\n"
        return output


    def display_info(self):
        print(f"Debug: CommandConstant: display_info")
        print(f" name: {self.name}, type: {self.type}, value: {self.value}, comment: {self.comment}, line: {self.line_number}")
        print(f"   description: {self.description}\n  param1: {self.param1}\n  param2: {self.param2}\n  param3: {self.param3}\n  param4: {self.param4}\n  param5: {self.param5}\n  param6: {self.param6}\n  param7: {self.param7}")

class MessageField:
    """
    Represents a field.
    Encapsulates parsing of the field information.
    """
    def __init__(self, name, type, comment, line_number, parentMessage):
        self.name = name
        self.type = type
        self.comment = comment
        self.unit = None
        self.enums = None
        self.minValue = None
        self.maxValue = None
        self.invalidValue = None
        self.frameValue = None
        self.lineNumber = line_number
        self.parent = parentMessage

        #print(f"MessageComment: {comment}")
        match = None
        if self.comment:
            match = re.match(r'^((?:\[[^\]]*\]\s*)+)(.*)$', comment)
        self.description = comment
        bracketed_part = None
        if match:
            bracketed_part = match.group(1).strip() # .strip() removes trailing whitespace from the bracketed part
            self.description = match.group(2).strip()
        if bracketed_part:
          # get units
            bracket_content_matches = re.findall(r'\[(.*?)\]', bracketed_part)
            #print(f"bracket_content_matches: {bracket_content_matches}")
            for item in bracket_content_matches:
                item = item.strip()
                if item.startswith('@'): # Not a unit:
                    if item.startswith('@enum'):
                        item = item.split(" ")
                        self.enums = item[1:]
                        # Create parent enum objects
                        for enumName in self.enums:
                            if not enumName in parentMessage.enums:
                                parentMessage.enums[enumName]=Enum(enumName,parentMessage)
                    elif item.startswith('@range'):
                        item = item[6:].strip().split(",")
                        self.minValue = item[0].strip()
                        self.maxValue = item[1].strip()
                    elif item.startswith('@invalid'):
                        self.invalidValue = item[8:].strip()
                        #TODO: Do we require a description? (not currently)
                        if self.invalidValue.split(" ")[0] not in ALLOWED_INVALID_VALUES:
                            error = Error("unknown_invalid_value", self.parent.filename, self.lineNumber, self.invalidValue, self.name)
                            #error.display_error()
                            if not "unknown_invalid_value" in self.parent.errors:
                                self.parent.errors["unknown_invalid_value"] = []
                            self.parent.errors["unknown_invalid_value"].append(error)
                    elif item.startswith('@frame'):
                        self.frameValue = item[6:].strip()
                        if self.frameValue not in ALLOWED_FRAMES:
                            error = Error("unknown_frame", self.parent.filename, self.lineNumber, self.frameValue, self.name)
                            #error.display_error()
                            if not "unknown_frame" in self.parent.errors:
                                self.parent.errors["unknown_frame"] = []
                            self.parent.errors["unknown_frame"].append(error)
                    else:
                        print(f"WARNING: Unhandled metadata in message comment: {item}")
                        # TODO - report errors for different kinds of metadata
                        exit()

                else: # bracket is a unit
                    self.unit = item

                    if self.unit not in ALLOWED_UNITS:
                        invalid_units.add(self.unit)
                        error = Error("unknown_unit", self.parent.filename, self.lineNumber, self.unit, self.name)
                        #error.display_error()
                        if not "unknown_unit" in self.parent.errors:
                            self.parent.errors["unknown_unit"] = []
                        self.parent.errors["unknown_unit"].append(error)

                    if item == "-":
                        self.unit = ""


    def display_info(self):
        print(f"Debug: MessageField: display_info")
        print(f" name: {self.name}, type: {self.type}, description: {self.description}, enums: {self.enums}, minValue: {self.minValue}, maxValue: {self.maxValue}, invalidValue: {self.invalidValue}, frameValue: {self.frameValue}")


class UORBMessage:
    """
    Represents a whole message, including fields, enums, commands, constants.
    The parser function delegates the parsing of each part of the message to
    more appropriate classes, once the specific type of line has been identified.
    """

    def __init__(self, filename):

        self.filename = filename
        msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../msg")
        self.msg_filename = os.path.join(msg_path, self.filename)
        self.name = os.path.splitext(os.path.basename(msg_file))[0]
        self.shortDescription = ""
        self.longDescription = ""
        self.fields = []
        self.constantFields = dict()
        self.commandConstants = dict()
        self.enums = dict()
        self.output_file = os.path.join(output_dir, f"{self.name}.md")
        self.topics = []
        self.errors = dict()

        self.parseFile()

        if args.errors:
            #print(f"DEBUG: args.errors: {args.errors}")
            if args.error_messages:
                messages = args.error_messages.split(" ")
                #print(f"DEBUG: args.errors: {messages},self.name: {self.name}")
                if self.name in messages:
                    self.reportErrors()
                    #print(f"Debug: {self.name} in {messages}")
            else:
              self.reportErrors()

    def reportErrors(self):
        #print(f"Debug: UORBMessage: reportErrors()")
        for errorType, errors in self.errors.items():
            for error in errors:
               error.display_error()

    def markdown_out(self):
        #print(f"Debug: UORBMessage: markdown_out()")

        # Add page header (forces wide pages)
        markdown = f"""---
pageClass: is-wide-page
---

# {self.name} (UORB message)

"""
        ## Append description info if present
        markdown += f"{self.shortDescription}\n\n" if self.shortDescription else ""
        markdown += f"{self.longDescription}\n\n" if self.longDescription else ""

        topicList = " ".join(self.topics)
        markdown += f"**TOPICS:** {topicList}\n\n"

        # Generate field docs
        markdown += f"## Fields\n\n"
        markdown += "Name | Type | Unit [Frame] | Range/Enum | Description\n"
        markdown += "--- | --- | --- | --- | ---\n"
        for field in self.fields:
            unit = f"{field.unit}" if field.unit else ""
            frame = f"[{field.frameValue}]" if field.frameValue else ""
            unit = f"{unit} {frame}"
            unit.strip()
            unit = f" {unit}"

            value = " "
            if field.enums:
                value = ""
                for enum in field.enums:
                    value += f"[{enum}](#{enum})"
                value = value.strip()
                value = f"{value}"
            elif field.minValue or field.maxValue:
                value = f"[{field.minValue if field.minValue else '-'} : {field.maxValue if field.maxValue else '-' }]"

            description = f" {field.description}" if field.description else ""
            invalid = f" (Invalid: {field.invalidValue}) " if field.invalidValue else ""
            markdown += f"{field.name} | `{field.type}` |{unit}|{value}|{description}{invalid}\n"

        # Generate table for command docs
        if len(self.commandConstants) > 0:
            #print("DEBUGCOMMAND")
            markdown += f"\n## Commands\n\n"

            """
            markdown += "Name | Type | Value | Description\n"
            markdown += "--- | --- | --- |---\n"
            for name, command in self.commandConstants.items():
                description = f" {command.comment} " if enum.comment else " "
                markdown += f'<a id="#{name}"></a> {name} | `{command.type}` | {command.value} |{description}\n'
            """
            for commandConstant in self.commandConstants.values():
                #print(commandConstant)
                markdown += commandConstant.markdown_out()

        # Generate enum docs
        if len(self.enums) > 0:
            markdown += f"\n## Enums\n"

            for name, enum in self.enums.items():
                markdown += f"\n### {name} {{#{name}}}\n\n"

                markdown += "Name | Type | Value | Description\n"
                markdown += "--- | --- | --- | ---\n"

                for enumValueName, enumValue in enum.enumValues.items():
                    description = f" {enumValue.comment} " if enumValue.comment else " "
                    markdown += f'<a href="#{enumValueName}"></a> {enumValueName} | `{enumValue.type}` | {enumValue.value} |{description}\n'

        # Generate table for constants docs
        if len(self.constantFields) > 0:
            markdown += f"\n## Constants\n\n"
            markdown += "Name | Type | Value | Description\n"
            markdown += "--- | --- | --- |---\n"
            for name, enum in self.constantFields.items():
                description = f" {enum.comment} " if enum.comment else " "
                markdown += f'<a id="#{name}"></a> {name} | `{enum.type}` | {enum.value} |{description}\n'



        # Append msg contents to the end
        with open(self.msg_filename, 'r') as source_file:
            msg_contents = source_file.read()
            msg_contents = msg_contents.strip()

        #Format markdown using msg name, comment, url, contents.
        markdown += f"""

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/{self.filename})

::: details Click here to see original file

```c
{msg_contents}
```

:::
"""

        with open(self.output_file, 'w') as content_file:
            content_file.write(markdown)

        #exit()


    def display_info(self):
        print(f"UORBMessage: display_info")
        print(f" name: {self.name}")
        print(f" filename: {self.filename}, ")
        print(f" msg_filename: {self.msg_filename}, ")
        print(f"self.shortDescription: {self.shortDescription}")
        print(f"self.longDescription: {self.longDescription}")
        print(f"self.enums: {self.enums}")

        for enum, enumObject in self.enums.items():
            enumObject.display_info()

        # Output our data so far
        for field in self.fields:
            field.display_info()

        for enumvalue in self.constantFields:
            print(enumvalue)
            self.constantFields[enumvalue].display_info()

    def handleField(self, line, line_number, parentMessage):
        #print(f"debug: handleField: (line): \n {line}")
        # Note, here we know we don't have a comment or a topic.
        # We expect it to be a field.

        # Check field doesn't have leading whitespace (trailing spaces already checked)
        if line[:1].isspace(): # Returns True for ' ', '\t', '\n', '\r', etc.
            #print("First character is whitespace")
            error = Error("leading_whitespace_field_or_constant", self.filename, line_number, line)
            if not "leading_whitespace_field_or_constant" in self.errors:
                self.errors["leading_whitespace_field_or_constant"] = []
                self.errors["leading_whitespace_field_or_constant"].append(error)

        # Now we can parse the stripped line
        fieldOrConstant = line.strip()

        # Check that the field or constant has only single whitespace separators
        stripped_fieldOrConstant = re.sub(r'\s+', ' ', fieldOrConstant) # Collapse all spaces to a single space (LHS already stripped).
        if stripped_fieldOrConstant != fieldOrConstant:
            #print("Field/Constant has multiple whitespace characters") # Since the collapsed version shows them.
            error = Error("field_or_constant_has_multiple_whitepsace", self.filename, line_number, line)
            if not "field_or_constant_has_multiple_whitepsace" in self.errors:
                self.errors["field_or_constant_has_multiple_whitepsace"] = []
                self.errors["field_or_constant_has_multiple_whitepsace"].append(error)

        fieldOrConstant = stripped_fieldOrConstant



        comment = None
        if "#" in line:
            commentExtract = line.split("#", 1) # Split once on left-most '#'
            fieldOrConstant = commentExtract[0].strip()
            comment = commentExtract[-1].strip()

        if "=" not in fieldOrConstant:
            # Is a field:
            field = fieldOrConstant.split(" ")
            type = field[0].strip()
            name = field[1].strip()
            field = MessageField(name, type, comment, line_number, parentMessage)
            self.fields.append(field)
        else:
            temp = fieldOrConstant.split("=")
            value = temp[-1]
            typeAndName = temp[0].split(" ")
            type = typeAndName[0]
            name = typeAndName[1]
            if name.startswith("VEHICLE_CMD_") and parentMessage.name == 'VehicleCommand': #it's a command.
                #print(f"DEBUG: startswith VEHICLE_CMD_ {name}")
                commandConstant = CommandConstant(name, type, value, comment, line_number, parentMessage)
                #commandConstant.display_info()
                self.commandConstants[name]=commandConstant
            else: #it's a constant (or part of an enum)
                constantField = ConstantValue(name, type, value, comment, line_number)
                self.constantFields[name]=constantField


    def parseFile(self):
        initial_block_lines = []
        #stopping_token = None
        found_first_relevant_content = False
        gettingInitialComments = False
        gettingFields = False

        with open(self.msg_filename, 'r', encoding='utf-8') as uorbfile:
            lines = uorbfile.read().splitlines()
            for line_number, line in enumerate(lines, 1):

                if line != line.rstrip():
                    #print(f"[{self.filename}] Trailing whitespace on line {line_number}: XX{line}YY")
                    error = Error("trailing_whitespace", self.filename, line_number, line)
                    if not "trailing_whitespace" in self.errors:
                        self.errors["trailing_whitespace"] = []
                    self.errors["trailing_whitespace"].append(error)

                #print(f"line: {line}")
                stripped_line = re.sub(r'\s+', ' ', line).strip() # Collapse all spaces to a single space and strip stuff off end.
                #print(f"stripped_line: {stripped_line}")
                # TODO? Perhaps report whitespace if the size of those two is different and it is empty
                # Or perhaps we just fix it on request

                isEmptyLine = False if line.strip() else True
                if not found_first_relevant_content and isEmptyLine: #Empty line
                    #print(f"{self.filename}: Empty line at start of file: [{line_number}]\n {line}")
                    error = Error("empty_start_line", self.filename, line_number, line)
                    if not "empty_start_line" in self.errors:
                        self.errors["empty_start_line"] = []
                    self.errors["empty_start_line"].append(error)
                    #error.display_error()
                    continue
                if not found_first_relevant_content and not isEmptyLine:
                    found_first_relevant_content = True

                    if stripped_line.startswith("#"):
                        gettingInitialComments = True
                    else:
                        gettingInitialComments = False
                        gettingFields = True

                if gettingInitialComments and stripped_line.startswith("#"):
                    stripped_line=stripped_line[1:].strip()
                    #print(f"DEBUG: gettingInitialComments: comment line: {stripped_line}")
                    initial_block_lines.append(stripped_line)
                else:
                    gettingInitialComments = False
                    gettingFields = True #Getting fields and constants
                if gettingFields:
                    if isEmptyLine:
                        continue # empty line
                    if stripped_line.startswith("# TOPICS "):
                        stripped_line = stripped_line[9:]
                        stripped_line = stripped_line.split(" ")
                        self.topics+= stripped_line
                        # Note, default topic and topic errors handled after all lines parsed
                        continue
                    if stripped_line.startswith("#"):
                        # Its an internal comment
                        stripped_line=stripped_line[1:].strip()

                        if stripped_line:
                            #print(f"{self.filename}: Internal comment: [{line_number}]\n {line}")
                            error = Error("internal_comment", self.filename, line_number, line)
                            if not "internal_comment" in self.errors:
                                self.errors["internal_comment"] = []
                            self.errors["internal_comment"].append(error)
                        else:
                            #print(f"{self.filename}: Empty internal comment: [{line_number}]\n {line}")
                            error = Error("internal_comment_empty", self.filename, line_number, line)
                            if not "internal_comment_empty" in self.errors:
                                self.errors["internal_comment_empty"] = []
                            self.errors["internal_comment_empty"].append(error)
                            #pass # Empty comment
                        continue

                    # Must be a field or a comment.
                    self.handleField(line, line_number, parentMessage=self)

            # Fix up topics if the topic is empty
            def camel_to_snake(name):
                # Insert underscore between lowercase/digit and uppercase letter
                s1 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', name)
                # Insert underscore between consecutive uppercase and uppercase+lowercase
                return re.sub('([A-Z]+)([A-Z][a-z])', r'\1_\2', s1).lower()

            defaultTopic = camel_to_snake(self.name)
            if len(self.topics) == 0:
                # We have no topic declared, so set the default topic
                self.topics.append(defaultTopic)
            elif len(self.topics) == 1:
                # We have 1 topic declared - either it is default or there is some issue.
                if defaultTopic in self.topics:
                    # Declared topic is default topic
                    error = Error("topic_error", self.filename, "", f"WARNING: TOPIC {defaultTopic} unnecessarily declared for {self.name}")
                else:
                    # Declared topic is not default topic
                    error = Error("topic_error", self.filename, "", f"NOTE: TOPIC {self.topics[0]}: Only Declared topic is not default topic {defaultTopic} for {self.name}")
                if not "topic_error" in self.errors:
                    self.errors["topic_error"] = []
                    self.errors["topic_error"].append(error)
            elif len(self.topics) > 1:
                if defaultTopic not in self.topics:
                    error = Error("topic_error", self.filename, "", f"NOTE: TOPIC - Default topic {defaultTopic} for {self.name} not in {self.topics}")

            # Parse our short and long description
            #print(f"DEBUG: initial_block_lines: {initial_block_lines}")
            doingLongDescription = False
            for summaryline in initial_block_lines:
                if not self.shortDescription and summaryline.strip() == '':
                    continue
                if not doingLongDescription and not summaryline.strip() == '':
                   self.shortDescription += f" {summaryline}"
                   self.shortDescription = self.shortDescription.strip()
                   if not self.shortDescription[-1:] == ".": # Add terminating fullstop if not present.
                       self.shortDescription += "."
                if not doingLongDescription and summaryline.strip() == '':
                   doingLongDescription = True
                   continue
                if doingLongDescription:
                    self.longDescription += f"{summaryline}\n"

            if self.longDescription:
                self.longDescription.strip()

            if not self.shortDescription:
                # Summary has not been defined
                error = Error("summary_missing", self.filename)
                if not "summary_missing" in self.errors:
                    self.errors["summary_missing"] = []
                self.errors["summary_missing"].append(error)


            # TODO Parse our constantValues into enums, leaving only constants
            constantValuesToRemove = []
            #print(f"DEBUG: Self.enums: {self.enums}")
            for enumName, enumObject in self.enums.items():
                for enumValueName, enumValueObject in self.constantFields.items():
                    if enumValueName.startswith(enumName):
                        # Copy this value into the object (cant be duplicate because parent is dict)
                        enumObject.enumValues[enumValueName]=enumValueObject
                        constantValuesToRemove.append(enumValueName)
            # Now delete the original enumvalues
            for enumValName in constantValuesToRemove:
                del self.constantFields[enumValName]
            constantsNotAssignedToEnums = len(self.constantFields)
            if constantsNotAssignedToEnums > 0:
                #print(f"Debug: WARNING constantsNotAssignedToEnums: {constantsNotAssignedToEnums}")
                for enumValueName, enumValue in self.constantFields.items():
                    if enumValueName in ALLOWED_CONSTANTS_NOT_IN_ENUM: # Ignore constants
                        pass
                    else:
                        error = Error("constant_not_in_assigned_enum", self.filename, enumValue.line_number, enumValueName)
                        if not "constant_not_in_assigned_enum" in self.errors:
                            self.errors["constant_not_in_assigned_enum"] = []
                        self.errors["constant_not_in_assigned_enum"].append(error)
                # TODO Maybe present as list of possible enums.


import yaml

def generate_dds_yaml_doc(allMessageFiles, output_file = 'dds_topics.md'):
    """
    Generates human readable version of dds_topics.yaml.
    Default output is to docs/en/middleware/dds_topics.md
    """

    dds_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../src/modules/uxrce_dds_client/dds_topics.yaml")
    output_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),f"../../docs/en/middleware/{output_file}")

    try:
        with open(dds_file_path, 'r') as file:
            data = yaml.safe_load(file)

        # Get messages and topics that are not published by default
        # Start by getting all that are published.
        all_messages_in_source = set()
        all_message_types =set()
        all_topics =set()
        for message in data["publications"]:
            all_message_types.add(message['type'].split("::")[-1])
            all_topics.add(message['topic'].split('/')[-1])
        for message in data["subscriptions"]:
            all_message_types.add(message['type'].split("::")[-1])
            all_topics.add(message['topic'].split('/')[-1])
        for message in (data.get("subscriptions_demux") or []):
            all_message_types.add(message['type'].split("::")[-1])
            all_topics.add(message['topic'].split('/')[-1])
        for message in allMessageFiles:
            all_messages_in_source.add(message.split('/')[-1].split('.')[0])
        messagesNotExported = all_messages_in_source - all_message_types

        # write out the dds file
        dds_markdown="""# dds_topics.yaml — PX4 Topics Exposed to ROS 2

::: info
This document is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::


The [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml) file specifies which uORB message definitions are compiled into the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) module when [PX4 is built](../middleware/uxrce_dds.md#code-generation), and hence which topics are available for ROS 2 applications to subscribe or publish (by default).

This document shows a markdown-rendered version of [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), listing the publications, subscriptions, and so on.

## Publications

Topic | Type| Rate Limit
--- | --- | ---
"""

        for message in data["publications"]:
            type = message['type']
            px4Type=type.split("::")[-1]
            dds_markdown += f"`{message['topic']}` | [{type}](../msg_docs/{px4Type}.md) | {message.get('rate_limit','')}\n"

        dds_markdown += "\n## Subscriptions\n\nTopic | Type\n--- | ---\n"

        for message in data["subscriptions"]:
            type = message['type']
            px4Type=type.split("::")[-1]
            dds_markdown += f"{message['topic']} | [{type}](../msg_docs/{px4Type}.md)\n"

        dds_markdown += "\n## Subscriptions Demux\n\n"

        subscriptions_demux = data.get("subscriptions_demux") or []
        if not subscriptions_demux:
            dds_markdown += "None\n"
        else:
            dds_markdown += "Topic | Type | Route Field | Max Instances\n--- | --- | --- | ---\n"
            for message in subscriptions_demux:
                type = message['type']
                px4Type = type.split("::")[-1]
                route_field = f"`{message['route_field']}`" if 'route_field' in message else "-"
                max_instances = message.get('max_instances', '-')
                dds_markdown += f"{message['topic']} | [{type}](../msg_docs/{px4Type}.md) | {route_field} | {max_instances}\n"

        if messagesNotExported:
            # Print the topics that are not exported to DDS
            dds_markdown += "\n## Not Exported\n\nThese messages are not listed in the yaml file.\nThey are not build into the module, and hence are neither published or subscribed."
            dds_markdown += "\n\n::: details See messages\n"
            for item in  messagesNotExported:
                dds_markdown += f"\n- [{item}](../msg_docs/{item}.md)"
            dds_markdown += "\n:::\n" # End of details block

        #print(dds_markdown)
        with open(output_file_path, 'w') as content_file:
                content_file.write(dds_markdown)

    except yaml.YAMLError as exc:
        print(f"Error parsing YAML: {exc}")
    except FileNotFoundError:
        print(f"Error: {dds_file_path} not found.")


def get_msgs_list(msgdir):
    """
    Makes a list of relative paths of .msg files in the given directory
    and its subdirectories.

    Parameters:
    msgdir (str): The directory to search for .msg files.

    Returns:
    list: A list of relative paths to .msg files.
    """
    msgs = []
    for root, _, files in os.walk(msgdir):
        for fn in files:
            if fn.endswith(".msg"):
                relative_path = os.path.relpath(os.path.join(root, fn), msgdir)
                msgs.append(relative_path)
    return msgs


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Generate docs from .msg files')
    parser.add_argument('-d', dest='dir', help='output directory', required=True)
    parser.add_argument('-e', dest='errors', action='store_true', help='Report errors')
    parser.add_argument('-m', dest='error_messages', help='Message to report errors against (by default all)')
    args = parser.parse_args()

    output_dir = args.dir
    if not os.path.isdir(output_dir):
        print(f"making output_dir {output_dir}")
        os.mkdir(output_dir)

    msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../msg")
    msg_files = get_msgs_list(msg_path)

    msg_files.sort()

    versioned_msgs_list = ''
    unversioned_msgs_list = ''
    msgTypes = set()

    for msg_file in msg_files:
        # Add messages to set of allowed types (compound types)
        #msg_type = msg_file.rsplit('/')[-1]
        #msg_type = msg_type.rsplit('\\')[-1]
        #msg_type = msg_type.rsplit('.')[0]
        msg_name = os.path.splitext(os.path.basename(msg_file))[0]
        msgTypes.add(msg_name)

    for msg_file in msg_files:
        message = UORBMessage(msg_file)
        # Any additional tests that can't be in UORBMessage parser go here.
        message.markdown_out()

        # Categorize as versioned or unversioned
        if "versioned" in msg_file:
            versioned_msgs_list += f"- [{message.name}]({message.name}.md)"
            if message.shortDescription:
                versioned_msgs_list += f" — {message.shortDescription}"
            versioned_msgs_list += "\n"
        else:
            unversioned_msgs_list += f"- [{message.name}]({message.name}.md)"
            if message.shortDescription:
                unversioned_msgs_list += f" — {message.shortDescription}"
            unversioned_msgs_list += "\n"
    # Write out the index.md file
    index_text=f"""# uORB Message Reference

::: info
This list is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

This topic lists the UORB messages available in PX4 (some of which may be may be shared by the [PX4-ROS 2 Bridge](../ros/ros2_comm.md)).

[Versioned messages](../middleware/uorb.md#message-versioning) track changes to their definitions, with each modification resulting in a version increment.
These messages are most likely shared through the PX4-ROS 2 Bridge.

Graphs showing how these are used [can be found here](../middleware/uorb_graph.md).

## Versioned Messages

{versioned_msgs_list}

## Unversioned Messages

{unversioned_msgs_list}
    """
    index_file = os.path.join(output_dir, 'index.md')
    with open(index_file, 'w', encoding='utf-8') as content_file:
            content_file.write(index_text)

    generate_dds_yaml_doc(msg_files)
