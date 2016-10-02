#!/usr/bin/python

import glob
import os
import sys
import re
from string import Template

# This script is run from Build/<target>_default.build/$(PX4_BASE)/Firmware/src/systemcmds/topic_listener

# argv[1] must be the full path of the top Firmware dir

raw_messages = glob.glob(sys.argv[1]+"/msg/*.msg")
messages = []
message_elements = []


for index,m in enumerate(raw_messages):
    temp_list_floats = []
    temp_list_uint64 = []
    temp_list_bool = []
    if("pwm_input" not in m and "position_setpoint" not in m):
        temp_list = []
        f = open(m,'r')
        for line in f.readlines():
            items = re.split('\s+', line.strip())

            if ('float32[' in items[0]):
                num_floats = int(items[0].split("[")[1].split("]")[0])
                temp_list.append(("float_array",items[1],num_floats))
            elif ('float64[' in items[0]):
                num_floats = int(items[0].split("[")[1].split("]")[0])
                temp_list.append(("double_array",items[1],num_floats))
            elif ('uint64[' in items[0]):
                num_floats = int(items[0].split("[")[1].split("]")[0])
                temp_list.append(("uint64_array",items[1],num_floats))
            elif(items[0] == "float32"):
                temp_list.append(("float",items[1]))
            elif(items[0] == "float64"):
                temp_list.append(("double",items[1]))
            elif(items[0] == "uint64") and len(line.split('=')) == 1:
                temp_list.append(("uint64",items[1]))
            elif(items[0] == "uint32") and len(line.split('=')) == 1:
                temp_list.append(("uint32",items[1]))
            elif(items[0] == "uint16") and len(line.split('=')) == 1:
                temp_list.append(("uint16",items[1]))
            elif(items[0] == "int64") and len(line.split('=')) == 1:
                temp_list.append(("int64",items[1]))
            elif(items[0] == "int32") and len(line.split('=')) == 1:
                temp_list.append(("int32",items[1]))
            elif(items[0] == "int16") and len(line.split('=')) == 1:
                temp_list.append(("int16",items[1]))
            elif (items[0] == "bool") and len(line.split('=')) == 1:
                temp_list.append(("bool",items[1]))
            elif (items[0] == "uint8") and len(line.split('=')) == 1:
                temp_list.append(("uint8",items[1]))
            elif (items[0] == "int8") and len(line.split('=')) == 1:
                temp_list.append(("int8",items[1]))

        f.close()
        (m_head, m_tail) = os.path.split(m)
        message = m_tail.split('.')[0]
        if message != "actuator_controls":
            messages.append(message)
            message_elements.append(temp_list)
        #messages.append(m.split('/')[-1].split('.')[0])

num_messages = len(messages);

include_topics = ""
for m in messages:
    include_topics += ("#include <uORB/topics/%s.h>\n" % m)

topic_classes = ""
factory = ""


for index,m in enumerate(messages[1:]):

    if index > 0:
        factory += "\telse "
    else:
        factory += "\t"
    factory += "if (strncmp(name, \"{}\",50) == 0) {{\n".format(m)
    factory += "\t\ttopic = new Topic_{}(ORB_ID({}));\n".format(m, m)
    factory += "\t}\n"

    topic_classes += "class Topic_{} : public Topic {{\n".format(m)
    topic_classes += "public:\n"
    topic_classes += "\tTopic_{}(orb_id_t id)\n".format(m)
    topic_classes += "\t\t: Topic(id),\n"
    topic_classes += "\t_container{}\n"
    topic_classes += "{\n"
    topic_classes += "}\n"

    print_function = ""
    print_function += ("\t\tPX4_INFO_RAW(\"timestamp: %\" PRIu64 \"\\n\", _container.timestamp);\n")


    for item in message_elements[index+1]:
        if item[0] == "float":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%8.4f\\n\",(double)_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "float_array":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: \");\n" % item[1])
            print_function += ("\t\tfor (int j = 0; j < %d; ++j) {\n" % item[2])
            print_function += ("\t\t\tPX4_INFO_RAW(\"%%8.4f \",(double)_container.%s[j]);\n" % item[1])
            print_function += ("\t\t}\n")
            print_function += ("\t\tPX4_INFO_RAW(\"\\n\");\n")
        elif item[0] == "double":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%8.4f\\n\",(double)_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "double_array":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: \");\n" % item[1])
            print_function += ("\t\tfor (int j = 0; j < %d; ++j) {\n" % item[2])
            print_function += ("\t\t\tPX4_INFO_RAW(\"%%8.4f \",(double)_container.%s[j]);\n" % item[1])
            print_function += ("\t\t}\n")
            print_function += ("\t\tPX4_INFO_RAW(\"\\n\");\n")
        elif item[0] == "uint64":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%\" PRIu64 \"\\n\",_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "uint64_array":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: \");\n" % item[1])
            print_function += ("\t\tfor (int j = 0; j < %d; ++j) {\n" % item[2])
            print_function += ("\t\t\tPX4_INFO_RAW(\"%%\" PRIu64 \"\",_container.%s[j]);\n" % item[1])
            print_function += ("\t\t}\n")
            print_function += ("\t\tPX4_INFO_RAW(\"\\n\");\n")
        elif item[0] == "int64":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%\" PRId64 \"\\n\",_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "int32":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%d\\n\",_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "uint32":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%u\\n\",_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "int16":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%d\\n\",(int)_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "uint16":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%u\\n\",(unsigned)_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "int8":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%d\\n\",(int)_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "uint8":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%u\\n\",(unsigned)_container.%s);\n" % (item[1], item[1]))
        elif item[0] == "bool":
            print_function += ("\t\tPX4_INFO_RAW(\"%s: %%s\\n\",_container.%s ? \"True\" : \"False\");\n" % (item[1], item[1]))

    topic_classes += "\tvoid print_specific() override\n"
    topic_classes += "\t{\n"
    topic_classes += print_function
    topic_classes += "\t}\n"

    topic_classes += "\tbool update() override"
    topic_classes += "\t{\n"
    topic_classes += "\t\tbool updated = false;\n"
    topic_classes += "\t\torb_check(_sub, &updated);\n"
    topic_classes += "\t\tif (updated) {\n"
    topic_classes += "\t\t\torb_copy(_id, _sub, &_container);\n"
    topic_classes += "\t\t\treturn true;\n"
    topic_classes += "\t\t} else {\n"
    topic_classes += "\t\t\treturn false;\n"
    topic_classes += "\t\t}\n"
    topic_classes += "\t}\n"


    topic_classes += "private:\n"
    topic_classes += "\tstruct {}_s _container;\n".format(m)
    topic_classes += "};\n\n"

with open("/home/julianoes/src/Firmware/src/systemcmds/topic_listener/topic_listener.cpp.in", "r") as f:
    template = Template(f.read())

substituted = template.substitute(include_topics=include_topics, topic_classes=topic_classes, factory=factory)

print(substituted)
