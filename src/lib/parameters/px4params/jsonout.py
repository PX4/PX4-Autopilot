from xml.sax.saxutils import escape
import codecs
import json
import sys


class JsonOutput():
    def __init__(self, groups, board, inject_xml_file_name):
        all_json=dict()
        all_json['version']=1
        all_params=[]
        all_json['parameters']=all_params

        schema_map = {
                        "short_desc": "shortDesc",
			"long_desc": "longDesc",
			"unit": "units",
			}
        schema_map_typed = {
			"min": "min",
			"max": "max",
			"increment": "increment",
			}
        schema_map_fix_type = {
			"reboot_required": ("rebootRequired", bool),
			"decimal": ("decimalPlaces", int),
			}
        allowed_types = { "Uint8", "Int8", "Uint16", "Int16", "Uint32", "Int32", "Float"}

        last_param_name = ""
        board_specific_param_set = False
        for group in groups:
            group_name=group.name

            def get_typed_value(value: str, type_name: str):
                if type_name == 'Float': return float(value)
                if type_name == 'Int32': return int(value)
                return value

            for param in sorted(group.parameters):
                if (last_param_name == param.name and not board_specific_param_set) or last_param_name != param.name:
                    curr_param=dict()
                    curr_param['name'] = param.name
                    type_name = param.type.capitalize()
                    curr_param['type'] = type_name
                    if not type_name in allowed_types:
                        print("Error: %s type not supported: curr_param['type']" % (curr_param['name'],curr_param['type']) )
                        sys.Exit(1)
                    curr_param['default'] = get_typed_value(param.default, type_name)

                    curr_param['group'] = group_name
                    if param.category != "":
                        curr_param['category'] = param.category
                    else:
                        curr_param['category'] = 'Standard'

                    if param.volatile:
                        curr_param['volatile'] = True

                    last_param_name = param.name
                    for code in param.field_keys:
                        value = param.fields.get(code, "")
                        # XML parsing returns empty fields as None
                        if value is None:
                            value = ""
                        if code == "board":
                            if value == board:
                                board_specific_param_set = True
                                # JSON schema has no field for board_specific schema. Ignore.
                            else:
                                #xml_group.remove(xml_param)
                                continue
                        else:
                            #map PX4 param field names to schema names
                            if code in schema_map:
                                curr_param[schema_map[code]] = value
                            elif code in schema_map_typed:
                                curr_param[schema_map_typed[code]] = get_typed_value(value, type_name)
                            elif code in schema_map_fix_type:
                                curr_param[schema_map_fix_type[code][0]] = schema_map_fix_type[code][1](value)
                            else:
                                print('ERROR: Field not in json schema: %s' % code)
                                sys.exit(1)


                if last_param_name != param.name:
                    board_specific_param_set = False

                enum_codes=sorted(param.enum.keys(), key=float)
                if enum_codes:
                    enum_codes=sorted(enum_codes,key=float)
                    codes_list=list()
                    for item in enum_codes:
                        code_dict=dict()
                        code_dict['value']=get_typed_value(item, type_name)
                        code_dict['description']=param.enum.get(item, "")
                        codes_list.append(code_dict)
                    curr_param['values'] = codes_list
                elif param.boolean:
                    curr_param['values'] = [
                        { 'value': 0, 'description': 'Disabled' },
                        { 'value': 1, 'description': 'Enabled' }
                    ]


                if len(param.bitmask.keys()) > 0:
                    bitmasks_list=list()
                    for index in sorted(param.bitmask.keys(), key=float):
                        bitmask_dict=dict()
                        bitmask_dict['index']=int(index)
                        bitmask_dict['description']=param.bitmask.get(index, "")
                        bitmasks_list.append(bitmask_dict)
                    curr_param['bitmask'] = bitmasks_list


                all_params.append(curr_param)


        #Json string output.
        self.output = json.dumps(all_json,indent=2)


    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)

