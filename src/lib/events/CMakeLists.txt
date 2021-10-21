############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_git_submodule(TARGET git_libevents PATH "libevents")

get_property(all_px4_src_files GLOBAL PROPERTY PX4_SRC_FILES)

set(generated_events_dir ${PX4_BINARY_DIR}/events)
set(generated_events_px4_file ${generated_events_dir}/px4.json)
set(generated_events_common_enums_file ${generated_events_dir}/common_with_enums.json)
add_custom_command(OUTPUT ${generated_events_px4_file}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_events_dir}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_events.py
		--src-path ${all_px4_src_files}
		--json ${generated_events_px4_file} #--verbose
	DEPENDS
		${all_px4_src_files}
		${PX4_SOURCE_DIR}/Tools/px_process_events.py
		${PX4_SOURCE_DIR}/Tools/px4events/jsonout.py
		${PX4_SOURCE_DIR}/Tools/px4events/srcparser.py
		${PX4_SOURCE_DIR}/Tools/px4events/srcscanner.py
		prebuild_targets # ensure all generated source files exist
	COMMENT "Generating px4 event json file from source"
)
add_custom_target(events_px4_json DEPENDS ${generated_events_px4_file})

set(generated_events_file ${generated_events_dir}/all_events.json)
add_custom_command(OUTPUT ${generated_events_file} ${generated_events_file}.xz
	COMMAND ${PYTHON_EXECUTABLE} libevents/scripts/combine.py
		${generated_events_px4_file}
		${generated_events_common_enums_file}
		--output ${generated_events_file}
	COMMAND ${PYTHON_EXECUTABLE} libevents/scripts/validate.py
		${generated_events_file}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/compress.py
		${generated_events_file}
	DEPENDS
		${generated_events_px4_file}
		${generated_events_common_enums_file}
		libevents/scripts/combine.py
		libevents/scripts/validate.py
		${PX4_SOURCE_DIR}/Tools/compress.py
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMENT "Generating combined event json file"
)
add_custom_target(events_json DEPENDS ${generated_events_file})

# combine common.json with our enums for the code generation
add_custom_command(OUTPUT ${generated_events_common_enums_file}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_events_dir}
	COMMAND ${PYTHON_EXECUTABLE} libevents/scripts/combine.py
		enums.json
		libevents/events/common.json
		--output ${generated_events_common_enums_file}
	DEPENDS
		enums.json
		libevents/events/common.json
		libevents/scripts/combine.py
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMENT "Generating combined event json file"
)
#add_custom_target(events_common_enums DEPENDS ${generated_events_common_enums_file})

# generate header from common.json + our enums
# (we don't add the definitions from the source as they are not needed and to
# avoid additional dependencies)
set(generated_events_header ${generated_events_dir}/events_generated.h)
add_custom_command(OUTPUT ${generated_events_header}
	COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_events_dir}
	COMMAND ${PYTHON_EXECUTABLE} libevents/scripts/generate.py
		--template libevents/libs/cpp/templates/events_generated.h.jinja
		--output ${generated_events_dir}
		${generated_events_common_enums_file}
	DEPENDS
		${generated_events_common_enums_file}
		libevents/libs/cpp/templates/events_generated.h.jinja
		libevents/scripts/generate.py
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMENT "Generating px4 event header file"
)
add_custom_target(events_header DEPENDS ${generated_events_header})
add_dependencies(prebuild_targets events_header)

