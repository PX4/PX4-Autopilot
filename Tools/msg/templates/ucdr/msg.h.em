@###############################################
@#
@# EmPy template
@#
@###############################################
@# generates CDR serialization & deserialization methods
@#
@# Context:
@#  - file_name_in (String) Source file
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@#  - search_path (dict) search paths for genmsg
@#  - topics (List of String) topic names
@###############################################
/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

@{
import genmsg.msgs
import re
from px_generate_uorb_topic_helper import * # this is in Tools/

topic = name_snake_case
uorb_struct = '%s_s'%name_snake_case

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
            name_snake = re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()
            print('#include <uORB/ucdr/%s.h>'%(name_snake))
}@

static inline constexpr int ucdr_topic_size_@(topic)()
{
	return @(struct_size);
}

static inline bool ucdr_serialize_@(topic)(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const @(uorb_struct)& topic = *static_cast<const @(uorb_struct)*>(data);
@{
for field_type, field_name, field_size, padding in fields:
	if padding > 0:
		print('\tbuf.iterator += {:}; // padding'.format(padding))
		print('\tbuf.offset += {:}; // padding'.format(padding))

	print('\tstatic_assert(sizeof(topic.{0}) == {1}, "size mismatch");'.format(field_name, field_size))

	if field_type == 'uint64' and field_name == 'timestamp':
		print('\tconst uint64_t timestamp_adjusted = topic.timestamp + time_offset;')
		print('\tmemcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.{0}));'.format(field_name))

	elif field_type == 'uint64' and field_name == 'timestamp_sample':
		print('\tconst uint64_t timestamp_sample_adjusted = topic.timestamp_sample + time_offset;')
		print('\tmemcpy(buf.iterator, &timestamp_sample_adjusted, sizeof(topic.{0}));'.format(field_name))

	else:
		print('\tmemcpy(buf.iterator, &topic.{0}, sizeof(topic.{0}));'.format(field_name))

	print('\tbuf.iterator += sizeof(topic.{:});'.format(field_name))
	print('\tbuf.offset += sizeof(topic.{:});'.format(field_name))

}@
	return true;
}

static inline bool ucdr_deserialize_@(topic)(ucdrBuffer& buf, @(uorb_struct)& topic, int64_t time_offset = 0)
{
@{
for field_type, field_name, field_size, padding in fields:
	if padding > 0:
		print('\tbuf.iterator += {:}; // padding'.format(padding))
		print('\tbuf.offset += {:}; // padding'.format(padding))

	print('\tstatic_assert(sizeof(topic.{0}) == {1}, "size mismatch");'.format(field_name, field_size))
	print('\tmemcpy(&topic.{0}, buf.iterator, sizeof(topic.{0}));'.format(field_name))

	if field_type == 'uint64' and (field_name == 'timestamp' or field_name == 'timestamp_sample'):
		print('\tif (topic.{0} == 0) topic.{0} = hrt_absolute_time();'.format(field_name, field_name))
		print('\telse topic.{0} = math::min(topic.{0} - time_offset, hrt_absolute_time());'.format(field_name, field_name))

	print('\tbuf.iterator += sizeof(topic.{:});'.format(field_name))
	print('\tbuf.offset += sizeof(topic.{:});'.format(field_name))

}@
	return true;
}
