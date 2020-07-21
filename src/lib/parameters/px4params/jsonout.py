from xml.sax.saxutils import escape
import codecs
import json


class JsonOutput():
    def __init__(self, groups):
        result = ""
        all_json=dict()
        all_json['version']=1
        all_json['uid']=1
        all_json['scope']="Firmware"
        all_params=[]
        all_json['parameters']=all_params
        for group in groups:
            #result += '## %s\n\n' % group.GetName()
            group_name=group.GetName()
            for param in group.GetParams():
                curr_param=dict()
                curr_param['name'] = param.GetName()
                curr_param['type'] = param.GetType().capitalize()
                curr_param['group'] = group_name
                curr_param['category'] = param.GetCategory()
                curr_param['shortDescription'] = param.GetFieldValue("short_desc")
                curr_param['longDescription'] = param.GetFieldValue("long_desc")
                curr_param['units'] = param.GetFieldValue("unit")
                curr_param['defaultValue'] = param.GetDefault()

                curr_param['decimalPlaces'] = param.GetFieldValue("decimal")
                curr_param['minValue'] = param.GetFieldValue("min")
                curr_param['maxValue'] = param.GetFieldValue("max")
                curr_param['increment'] = param.GetFieldValue("increment")
                curr_param['rebootRequired'] = param.GetFieldValue("reboot_required")
                curr_param['volatile'] = param.GetVolatile()

                enum_codes=param.GetEnumCodes() or '' # Gets numerical values for parameter.
                if enum_codes:
                    enum_codes=sorted(enum_codes,key=float)
                    codes_list=list()
                    for item in enum_codes:
                        code_dict=dict()
                        code_dict['value']=item
                        code_dict['description']=param.GetEnumValue(item)
                        codes_list.append(code_dict)
                    #only add this if values are defined.
                    curr_param['values'] = codes_list



                all_params.append(curr_param)


        #Note clear if we need additionalProperties, required, and what to do if values not defined.

        #Json string output.
        self.output = json.dumps(all_json,indent=2)




    def Save(self, filename):
        with codecs.open(filename, 'w', 'utf-8') as f:
            f.write(self.output)
