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

echo_topics = ""

echo_topics += ("\tif (strncmp(argv[1],\"%s\",50) == 0) {\n" % messages[0])
echo_topics += ("\t\tsub = orb_subscribe(ORB_ID(%s));\n" % messages[0])
echo_topics += ("\t\tID = ORB_ID(%s);\n" % messages[0])
echo_topics += ("\t\tstruct %s_s container;\n" % messages[0])
echo_topics += ("\t\tmemset(&container, 0, sizeof(container));\n")

for index,m in enumerate(messages[1:]):
	echo_topics += ("\t} else if (strncmp(argv[1],\"%s\",50) == 0) {\n" % m)
	echo_topics += ("\t\tsub = orb_subscribe(ORB_ID(%s));\n" % m)
	echo_topics += ("\t\tID = ORB_ID(%s);\n" % m)
	echo_topics += ("\t\tstruct %s_s container;\n" % m)
	echo_topics += ("\t\tmemset(&container, 0, sizeof(container));\n")
	echo_topics += ("\t\tbool updated;\n")
	echo_topics += ("\t\tunsigned i = 0;\n")
	echo_topics += ("\t\thrt_abstime start_time = hrt_absolute_time();\n")
	echo_topics += ("\t\twhile(i < num_msgs) {\n")
	echo_topics += ("\t\t\torb_check(sub,&updated);\n")
	echo_topics += ("\t\t\tif (i == 0) { updated = true; } else { usleep(500); }\n")
	echo_topics += ("\t\t\tif (updated) {\n")
	echo_topics += ("\t\t\tstart_time = hrt_absolute_time();\n")
	echo_topics += ("\t\t\ti++;\n")
	echo_topics += ("\t\t\tPX4_INFO_RAW(\"\\nTOPIC: %s #%%d\\n\", i);\n" % m)
	echo_topics += ("\t\t\torb_copy(ID,sub,&container);\n")
	echo_topics += ("\t\t\tPX4_INFO_RAW(\"timestamp: %\" PRIu64 \"\\n\", container.timestamp);\n")
	for item in message_elements[index+1]:
		if item[0] == "float":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%8.4f\\n\",(double)container.%s);\n" % (item[1], item[1]))
		elif item[0] == "float_array":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: \");\n" % item[1])
			echo_topics += ("\t\t\tfor (int j = 0; j < %d; j++) {\n" % item[2])
			echo_topics += ("\t\t\t\tPX4_INFO_RAW(\"%%8.4f \",(double)container.%s[j]);\n" % item[1])
			echo_topics += ("\t\t\t}\n")
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"\\n\");\n")
		elif item[0] == "double":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%8.4f\\n\",(double)container.%s);\n" % (item[1], item[1]))
		elif item[0] == "double_array":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: \");\n" % item[1])
			echo_topics += ("\t\t\tfor (int j = 0; j < %d; j++) {\n" % item[2])
			echo_topics += ("\t\t\t\tPX4_INFO_RAW(\"%%8.4f \",(double)container.%s[j]);\n" % item[1])
			echo_topics += ("\t\t\t}\n")
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"\\n\");\n")
		elif item[0] == "uint64":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%\" PRIu64 \"\\n\",container.%s);\n" % (item[1], item[1]))
		elif item[0] == "uint64_array":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: \");\n" % item[1])
			echo_topics += ("\t\t\tfor (int j = 0; j < %d; j++) {\n" % item[2])
			echo_topics += ("\t\t\t\tPX4_INFO_RAW(\"%%\" PRIu64 \"\",container.%s[j]);\n" % item[1])
			echo_topics += ("\t\t\t}\n")
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"\\n\");\n")
		elif item[0] == "int64":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%\" PRId64 \"\\n\",container.%s);\n" % (item[1], item[1]))
		elif item[0] == "int32":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%d\\n\",container.%s);\n" % (item[1], item[1]))
		elif item[0] == "uint32":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%u\\n\",container.%s);\n" % (item[1], item[1]))
		elif item[0] == "int16":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%d\\n\",(int)container.%s);\n" % (item[1], item[1]))
		elif item[0] == "uint16":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%u\\n\",(unsigned)container.%s);\n" % (item[1], item[1]))
		elif item[0] == "int8":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%d\\n\",(int)container.%s);\n" % (item[1], item[1]))
		elif item[0] == "uint8":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%u\\n\",(unsigned)container.%s);\n" % (item[1], item[1]))
		elif item[0] == "bool":
			echo_topics += ("\t\t\tPX4_INFO_RAW(\"%s: %%s\\n\",container.%s ? \"True\" : \"False\");\n" % (item[1], item[1]))
	echo_topics += ("\t\t\t} else {\n")
	echo_topics += ("\t\t\t\tif (check_timeout(start_time)) {\n")
	echo_topics += ("\t\t\t\t\tbreak;\n")
	echo_topics += ("\t\t\t\t}\n")
	echo_topics += ("\t\t\t}\n")
	echo_topics += ("\t\t}\n")
echo_topics += ("\t} else {\n")
echo_topics += ("\t\t PX4_INFO_RAW(\" Topic did not match any known topics\\n\");\n")
echo_topics += ("\t}\n")

with open("/home/julianoes/src/Firmware/src/systemcmds/topic_listener/topic_listener.cpp.in", "r") as f:
    template = Template(f.read())

substituted = template.substitute(include_topics=include_topics, echo_topics=echo_topics)

print(substituted)
