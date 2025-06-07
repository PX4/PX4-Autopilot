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

ALLOWED_UNITS = set(["m", "m/s", "rad", "rad/s", "rpm" ,"V", "A", "mA", "mAh", "W", "dBm", "s", "ms", "us", "Ohm", "MB", "Kb/s"])
invalid_units = set()

class Enum:
    def __init__(self, name, parentMessage):
        self.name = name
        self.parent = parentMessage
        self.enumValues = dict()

    def display_info(self):
        print(f"Debug: Enum: display_info")
        print(f" name: {self.name}")
        for key, value in self.enumValues.items():
            value.display_info()

class MessageField:
    def __init__(self, name, type, comment, line_number, parentMessage):
        self.name = name
        self.type = type
        self.comment = comment
        self.unit = None
        self.enums = None
        self.minValue = None
        self.maxValue = None
        self.invalidValue = None
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
            bracket_content_matches = matches = re.findall(r'\[(.*?)\]', bracketed_part)
            #print(f"bracket_content_matches: {bracket_content_matches}")
            for item in bracket_content_matches:
                item = item.strip()
                if not item.startswith('@'): # a unit
                    self.unit = item
                    if self.unit not in ALLOWED_UNITS:
                        invalid_units.add(self.unit)
                        print(f"Invalid Unit [{self.unit}] on {self.name} ({self.parent.filename}: {self.lineNumber})")
                        # TODO turn this into an error or warning thingy stored at top level.
                        # Would allow filtering on the types of things to report by file.
                elif item.startswith('@enum'):
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
                else:
                    print(f"WARNING: Unhandled metadata in message comment: {item}")
                    exit()

    def display_info(self):
        print(f"Debug: MessageField: display_info")
        print(f" name: {self.name}, type: {self.type}, description: {self.description}, enums: {self.enums}, minValue: {self.minValue}, maxValue: {self.maxValue}, invalidValue: {self.invalidValue}")

class EnumValue:
    def __init__(self, name, type, value, comment, line_number):
        self.name = name.strip()
        self.type = type.strip()
        self.value = value.strip()
        self.comment = comment
        self.line_number = line_number

        if not self.value:
            print(f"WARNING: NO VALUE in enumValue: {self.name}")
            exit()

        # TODO if value or name are empty, error

    def display_info(self):
        print(f"Debug: EnumValue: display_info")
        print(f" name: {self.name}, type: {self.type}, value: {self.value}, comment: {self.comment}")

class UORBMessage:
    def __init__(self, filename):

        self.filename = filename
        msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../msg")
        self.msg_filename = os.path.join(msg_path, self.filename)
        self.name = os.path.splitext(os.path.basename(msg_file))[0]
        self.shortDescription = ""
        self.longDescription = ""
        self.fields = []
        self.enumValues = dict()
        self.enums = dict()
        self.output_file = os.path.join(output_dir, f"{self.name}.md")

        self.parseFile()


    def markdown_out(self):
        #print(f"Debug: UORBMessage: markdown_out()")

        markdown = f"# {self.name} (UORB message)\n\n"
        markdown += f"[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/{self.filename})\n\n"

        ## Append description info if present
        markdown += f"{self.shortDescription}\n\n" if self.shortDescription else ""
        markdown += f"{self.longDescription}\n\n" if self.longDescription else ""

        # Generate field docs
        markdown += f"## Fields\n\n"
        markdown += "Name (type) | Units | Values | Description | invalid\n"
        markdown += "--- | --- | --- | --- | ---\n"
        for field in self.fields:
            unit = f" {field.unit} " if field.unit else " "
            value = " "
            if field.enums:
                value = ""
                for enum in field.enums:
                    value += f"[{enum}](#{enum}) "
                value = value.strip()
                value = f" {value} "
            elif field.minValue and field.maxValue:
                value = f" range: {field.minValue} - {field.maxValue} "
            elif field.minValue:
                value = f" min: {field.minValue} "
            elif field.maxValue:
                value = f" max: {field.maxValue} "
            description = f" {field.description} " if field.description else " "
            invalid = f" {field.invalidValue} " if field.invalidValue else " "

            markdown += f"{field.name} (`{field.type}`) |{unit}|{value}|{description}|{invalid}\n"

        # Generate enum docs
        if len(self.enums) > 0:
            markdown += f"\n## Enums\n"

            for name, enum in self.enums.items():
                markdown += f"\n### {name} {{#{name}}}\n\n"

                markdown += "Name (type) | Value | Description\n"
                markdown += "--- | --- | ---\n"

                for enumValueName, enumValue in enum.enumValues.items():
                    description = f" {enumValue.comment} " if enumValue.comment else " "
                    markdown += f"{enumValueName} (`{enumValue.type}`) | {enumValue.value} |{description}\n"

        # Generate table for constants docs
        if len(self.enumValues) > 0:
            markdown += f"\n## Constants\n\n"
            markdown += "Name (type) | Value | Description\n"
            markdown += "--- | --- | ---\n"
            for name, enum in self.enumValues.items():
                description = f" {enum.comment} " if enum.comment else " "
                markdown += f'<a href="#{name}">{name} (`{enum.type}`) | {enum.value} |{description}\n'

        # Append msg contents to the end
        with open(self.msg_filename, 'r') as source_file:
            msg_contents = source_file.read()
            msg_contents = msg_contents.strip()

        #Format markdown using msg name, comment, url, contents.
        markdown += f"""

## Source

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

        for enumvalue in self.enumValues:
            print(enumvalue)
            self.enumValues[enumvalue].display_info()

    def handleField(self, line, line_number, parentMessage):
        #print(f"handleField: (line): \n XX{line}XX")
        fieldOrConstant = line.strip()
        comment = None
        if "#" in line:
            commentExtract = line.split("#") #TODO should check for multiples and take first
            fieldOrConstant = commentExtract[0].strip()
            comment = commentExtract[-1].strip()
        #print(f" Comment: {comment}")
        #print(f"fieldOrConstant: {fieldOrConstant}")

        if "=" not in fieldOrConstant:
            # Is constant:
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
            enumValue = EnumValue(name, type, value, comment, line_number)
            self.enumValues[name]=enumValue


    def parseFile(self):
        initial_block_lines = []
        #stopping_token = None
        found_first_relevant_content = False
        gettingInitialComments = False
        gettingFields = False

        with open(self.msg_filename, 'r', encoding='utf-8') as uorbfile:
            for line_number, line in enumerate(uorbfile, 1):
                #print(f"line: {line}")
                stripped_line = re.sub(r'\s+', ' ', line).strip()
                #print(f"stripped_line: {stripped_line}")
                # TODO? Perhaps report whitespace if the size of those two is different and it is empty
                # Or perhaps we just fix it on request

                if not found_first_relevant_content and not stripped_line:
                    continue
                if not found_first_relevant_content and stripped_line:
                    found_first_relevant_content = True

                    if stripped_line.startswith("#"):
                        gettingInitialComments = True
                    else:
                        gettingInitialComments = False
                        gettingFields = True

                if gettingInitialComments and stripped_line.startswith("#"):
                    stripped_line=stripped_line[1:].strip()
                    #print(f"comment line: {stripped_line}")
                    initial_block_lines.append(stripped_line)
                else:
                    gettingInitialComments = False
                    gettingFields = True
                if gettingFields:
                    if not stripped_line:
                        continue # empty line
                    if stripped_line.startswith("# TOPICS"):
                        print("WARNING - DONT HANDLE TOPICS YET") # TODO
                        continue
                    if stripped_line.startswith("#"):
                        stripped_line=stripped_line[1:].strip()
                        if not stripped_line:
                            pass # Empty comment
                        else:
                            print(f"{self.filename}: Internal comment: [{line_number}]\n {line}")
                            # TODO report error?
                        continue
                    else:
                        #print(f"Field? {stripped_line}")
                        self.handleField(stripped_line, line_number, parentMessage=self)

            # Parse our short and long description
            #print(f"TODO: initial_block_lines: {initial_block_lines}")
            doingLongDescription = False
            for summaryline in initial_block_lines:
                if not self.shortDescription and summaryline.strip() == '':
                    continue
                if not doingLongDescription and not summaryline.strip() == '':
                   self.shortDescription += f" {summaryline}"
                   self.shortDescription = self.shortDescription.strip()
                if not doingLongDescription and summaryline.strip() == '':
                   doingLongDescription = True
                   continue
                if doingLongDescription:
                    self.longDescription += f"{summaryline}\n"

            else:
                print("WARNING: No summary")
            if self.longDescription:
                self.longDescription.strip()

            # TODO Parse our enumvalues into enums, leaving only constants
            enumValuesToRemove = []
            for enumName, enumObject in self.enums.items():
                #print(f"enum enumName key: {enumName}")
                for enumValueName, enumValueObject in self.enumValues.items():
                    #print(f"enumValueName key: {enumValueName}")
                    if enumValueName.startswith(enumName):
                        # Copy this value into the object (cant be duplicate because parent is dict)
                        enumObject.enumValues[enumValueName]=enumValueObject
                        enumValuesToRemove.append(enumValueName)
            # Now delete the original enumvalues
            for enumValName in enumValuesToRemove:
                del self.enumValues[enumValName]
            unassignedEnumValues = len(self.enumValues)
            if unassignedEnumValues > 0:
                print(f"Debug: WARNING unassignedEnumValues: {unassignedEnumValues}")
                # TODO Attempt to work out name of enum and report error.

            #self.display_info()


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
        if data["subscriptions_multi"]: # There is none now
            dds_markdown += "None\n"
            for message in data["subscriptions_multi"]:
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

        dds_markdown += "\n## Subscriptions Multi\n\n"

        if not data["subscriptions_multi"]: # There is none now
            dds_markdown += "None\n"
        else:
            print("Warning - we now have subscription_multi data - check format")
            dds_markdown += "Topic | Type\n--- | ---\n"
            for message in data["subscriptions_multi"]:
                dds_markdown += f"{message['topic']} | {message['type']}\n"

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
    parser.add_argument('-m', dest='messages', help='Message names, space separated, to run on specific messages')
    args = parser.parse_args()

    output_dir = args.dir
    if not os.path.isdir(output_dir):
        print(f"making output_dir {output_dir}")
        os.mkdir(output_dir)

    msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../msg")
    msg_files = get_msgs_list(msg_path)

    if args.messages:
        messageListToParse = args.messages.split(" ")
        print(messageListToParse)
        allFoundMessageNames = []

        for message in messageListToParse:
            messageFound = False
            for item in msg_files:
                if message in item:
                    allFoundMessageNames.append(item)
                    messageFound = True
                    continue
            if not messageFound:
                print(f"EXIT: Message not found: {message}")
                exit()
        print(f"Parsing: {allFoundMessageNames}")
        msg_files = allFoundMessageNames

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
    with open(index_file, 'w') as content_file:
            content_file.write(index_text)

    generate_dds_yaml_doc(msg_files)
