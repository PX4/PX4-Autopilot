@###############################################
@#
@# PX4 ROS compatible message source code
@# generation for C++
@#
@# EmPy template for generating <msg>.h files
@# Based on the original template for ROS
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - file_name_in (String) Source file
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@#  - search_path (dict) search paths for genmsg
@#  - topics (List of String) topic names
@#  - all_topics (List of String) all generated topic names (sorted)
@###############################################

@{
import genmsg.msgs
import json

from px_generate_uorb_topic_helper import * # this is in Tools/

uorb_struct = '%s_s'%name_snake_case

sorted_fields = sorted(spec.parsed_fields(), key=sizeof_field_type, reverse=True)
struct_size, padding_end_size = add_padding_bytes(sorted_fields, search_path)
topic_fields = ["%s %s" % (convert_type(field.type, True), field.name) for field in sorted_fields]

dependencies = []
for field in spec.parsed_fields():
    if not field.is_header:
        type_name = field.type
        # detect embedded types
        sl_pos = type_name.find('/')
        if sl_pos >= 0: # nested type
            dependencies.append(field.base_type)
}@

{
@# join all msg files in one line e.g: "float[3] position;float[3] velocity;bool armed;"
		"fields": @( json.dumps(bytearray(";".join(topic_fields)+";", 'utf-8').decode('unicode_escape')) ),
		"fields_total_length": @(sum([len(convert_type(field.type))+1+len(field.name)+1 for field in sorted_fields])),
		"orb_ids": @( json.dumps([ all_topics.index(topic) for topic in topics]) ),
		"main_orb_id": @( all_topics.index(name_snake_case) if name_snake_case in all_topics else -1 ),
		"dependencies": @( json.dumps(list(set(dependencies))) ),
		"name": "@( spec.full_name )"
}