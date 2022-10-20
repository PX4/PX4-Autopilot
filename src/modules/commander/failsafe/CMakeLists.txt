############################################################################
#
#   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

# Extract information from failsafe msg file
set(failsafe_flags_msg_file ${PX4_SOURCE_DIR}/msg/FailsafeFlags.msg)
set(generated_uorb_struct_field_mapping_header ${PX4_BINARY_DIR}/generated_uorb_struct_field_mapping.h)
set(html_template_file ${CMAKE_CURRENT_SOURCE_DIR}/emscripten_template.html)
set(html_output_file ${PX4_BINARY_DIR}/failsafe_html_template.html)
add_custom_command(OUTPUT ${generated_uorb_struct_field_mapping_header} ${html_output_file}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/parse_flags_from_msg.py
		${failsafe_flags_msg_file} ${generated_uorb_struct_field_mapping_header}
		${html_template_file} ${html_output_file}
	DEPENDS
		${failsafe_flags_msg_file}
		${html_template_file}
		${CMAKE_CURRENT_SOURCE_DIR}/parse_flags_from_msg.py
		COMMENT "Extracting info from failsafe flags msg file"
)
add_custom_target(failsafe_uorb_struct_header DEPENDS ${generated_uorb_struct_field_mapping_header})

# Webassembly failsafe testing (expects CMAKE_CXX_COMPILER = "em++")
add_executable(failsafe_test EXCLUDE_FROM_ALL
	emscripten.cpp
	failsafe.cpp
	framework.cpp
	)
set_property(TARGET failsafe_test PROPERTY UNITY_BUILD ON) # avoids some method calls to e.g. param_name or param_type
set_property(TARGET failsafe_test PROPERTY LINK_DEPENDS ${html_output_file})
add_dependencies(failsafe_test failsafe_uorb_struct_header uorb_headers parameters_header events_header mode_util)
target_compile_definitions(failsafe_test PUBLIC EMSCRIPTEN_BUILD PRINTF_LOG MODULE_NAME=\"failsafe\")
set_target_properties(failsafe_test PROPERTIES OUTPUT_NAME "index.html")

get_target_property(failsafe_test_compile_options failsafe_test COMPILE_OPTIONS)
list(REMOVE_ITEM failsafe_test_compile_options "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>")
set_property(TARGET failsafe_test PROPERTY COMPILE_OPTIONS ${failsafe_test_compile_options})

target_link_libraries(failsafe_test mode_util embind)
# TODO: use target_link_options() when updating to cmake 3.13 or newer
#target_link_options(failsafe_test PUBLIC --shell-file ${html_output_file} --no-entry -sLLD_REPORT_UNDEFINED -s NO_EXIT_RUNTIME=1)
target_link_libraries(failsafe_test
	"--shell-file ${html_output_file}"
	"--no-entry -sLLD_REPORT_UNDEFINED"
	"-s NO_EXIT_RUNTIME=1")

px4_add_library(failsafe
	failsafe.cpp
	framework.cpp
)

px4_add_functional_gtest(SRC failsafe_test.cpp
	LINKLIBS failsafe mode_util
)
