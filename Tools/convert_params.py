import sys
from pyparsing import *
import yaml

group = Literal("*").suppress() + Literal("@group") + Word(alphas)[1, ...] + LineEnd().suppress()
attribute = Literal('*').suppress() + Combine("@"+Word(alphas+"_")) + Word(alphanums+"./^-()") + LineEnd().suppress()
description = Literal('*').suppress() + Group(Word(alphanums+"/.,[]-_():=><%+'")[1, ...]) + \
    LineEnd().suppress()
empty_comment = Literal('*').suppress() + LineEnd().suppress()
comment_line = Group(group) | Group(attribute) | empty_comment.suppress()
description_line = description | empty_comment.suppress()
comment = Literal('/**').suppress() + Group(description_line[1, ...]) + \
    Group(comment_line[1, ...]) + Literal("*/").suppress()
param_macro = oneOf(["PARAM_DEFINE_FLOAT", "PARAM_DEFINE_INT32"])
param_name = Word(alphanums + "_/.")
param_value = Combine(Word(nums) + Optional('.') + Optional(Word(nums+"e+-")) + Optional("f"))
definition = Group(param_macro + Literal("(").suppress() + \
                   param_name + Literal(",").suppress() + param_value + \
                   Literal(");").suppress())
parameter = Group(comment + definition)
params = parameter[1, ...]

s = """/**
 * Board rotation Y (Pitch) offset
 *
 * This parameter defines a rotational offset in degrees around the Y (Pitch) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_BOARD_Y_OFF, 0.0f);"""

with open(sys.argv[1]) as f:
    text = f.read()

# text = text.split("****/")[1]
# text = text.split("*/", 1)[1]
text = text.strip()
# print(text)
# print("\n".join(text.split("\n")[:10]))
parsed = params.parseString(text)
# parsed = params.parseString(s)

output_params = []
groups = {}
for param in parsed:
    # print(param[2])
    gen = {"description": {}}
    if len(param[0]) == 2:
        gen["description"]["short"] = " ".join(param[0][0])
        gen["description"]["long"] = " ".join(param[0][1])
    else:
        gen["description"]["short"] = " ".join(param[0][0])
        gen["description"]["long"] = " ".join(param[0][0])
    group_name = ""
    for attr in param[1]:
        if attr[0] != "@group":
            try:
                gen[attr[0][1:]] = int(attr[1])
            except:
                try:
                    gen[attr[0][1:]] = float(attr[1])
                except:
                    gen[attr[0][1:]] = attr[1]
        else:
            group_name = attr[1]

    if param[2][0] == "PARAM_DEFINE_FLOAT":
        gen["type"] = "float"
        gen["default"] = float(param[2][2].rstrip('f'))
    elif param[2][0] == "PARAM_DEFINE_INT32":
        gen["type"] = "int32"
        gen["default"] = int(param[2][2])

    gen = {param[2][1]: gen}
    if group_name not in groups:
        groups[group_name] = []
    groups[group_name].append(gen)

    # print(gen)
    # print()

# print(groups)
output = {"parameters": []}
for group_name, group_items in groups.items():
    output["parameters"].append({"group": group_name, "definitions": group_items})

# print(output)
print(yaml.dump(output, default_flow_style=False, sort_keys=False))
# with open("module.yaml") as f:
    # print(yaml.load(f, Loader=yaml.FullLoader))
    # print(parameter.parseString(s))
