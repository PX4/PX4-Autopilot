@###############################################
@#
@# EmPy template
@#
@###############################################
@# generates CDR serialization & deserialization methods
@#
@# Context:
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@#  - search_path (str) Path to .msg files
@###############################################
@{
import genmsg.msgs
from px_generate_uorb_topic_helper import * # this is in Tools/

topic = spec.short_name
uorb_struct = '%s_s'%spec.short_name

# get fields, struct size and paddings
def add_fields(msg_fields, name_prefix='', offset=0):
	fields = []
	for field in msg_fields:
		if not field.is_header:
			field_size = sizeof_field_type(field)

			type_name = field.type
			# detect embedded types
			sl_pos = type_name.find('/')
			if (sl_pos >= 0):
				package = type_name[:sl_pos]
				type_name = type_name[sl_pos + 1:]

			# detect arrays
			a_pos = type_name.find('[')
			array_size = 1
			if (a_pos >= 0):
				# field is array
				array_size = int(type_name[a_pos+1:-1])
				type_name = type_name[:a_pos]

			if sl_pos >= 0: # nested type

				children_fields = get_children_fields(field.base_type, search_path)

				for i in range(array_size):
					sub_name_prefix = name_prefix+field.name
					if array_size > 1:
						sub_name_prefix += '['+str(i)+']'
					sub_fields, offset = add_fields(children_fields, sub_name_prefix+'.', offset)
					fields.extend(sub_fields)
			else:
				assert field_size > 0

				# note: the maximum alignment for XCDR is 8 and for XCDR2 it is 4
				padding = (field_size - (offset % field_size)) & (field_size - 1)

				fields.append((type_name, name_prefix+field.name, field_size * array_size, padding))
				offset += array_size * field_size + padding
	return fields, offset

fields, struct_size = add_fields(spec.parsed_fields())

}@

// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/@(topic).h>

@##############################
@# Includes for dependencies
@##############################
@{
for field in spec.parsed_fields():
    if (not field.is_builtin):
        if not field.is_header:
            (package, name) = genmsg.names.package_resource_name(field.base_type)
            package = package or spec.package # convert '' to package
            print('#include <uORB/ucdr/%s.h>'%(name))
}@

static inline constexpr int ucdr_topic_size_@(topic)()
{
	return @(struct_size);
}

bool ucdr_serialize_@(topic)(const @(uorb_struct)& topic, ucdrBuffer& buf)
{
	if (ucdr_buffer_remaining(&buf) < @(struct_size)) {
		return false;
	}
@{
for field_type, field_name, field_size, padding in fields:
	if padding > 0:
		print('\tbuf.iterator += {:}; // padding'.format(padding))
		print('\tbuf.offset += {:}; // padding'.format(padding))

	print('\tstatic_assert(sizeof(topic.{0}) == {1}, "size mismatch");'.format(field_name, field_size))
	print('\tmemcpy(buf.iterator, &topic.{0}, sizeof(topic.{0}));'.format(field_name))
	print('\tbuf.iterator += sizeof(topic.{:});'.format(field_name))
	print('\tbuf.offset += sizeof(topic.{:});'.format(field_name))

}@
	return true;
}

bool ucdr_deserialize_@(topic)(ucdrBuffer& buf, @(uorb_struct)& topic)
{
	if (ucdr_buffer_remaining(&buf) < @(struct_size)) {
		return false;
	}
@{
for field_type, field_name, field_size, padding in fields:
	if padding > 0:
		print('\tbuf.iterator += {:}; // padding'.format(padding))
		print('\tbuf.offset += {:}; // padding'.format(padding))

	print('\tstatic_assert(sizeof(topic.{0}) == {1}, "size mismatch");'.format(field_name, field_size))
	print('\tmemcpy(&topic.{0}, buf.iterator, sizeof(topic.{0}));'.format(field_name))
	print('\tbuf.iterator += sizeof(topic.{:});'.format(field_name))
	print('\tbuf.offset += sizeof(topic.{:});'.format(field_name))

}@
	return true;
}
