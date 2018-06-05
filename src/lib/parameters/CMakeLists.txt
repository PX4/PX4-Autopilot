############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

add_subdirectory(tinybson)

if (NOT PARAM_DEFAULT_OVERRIDES)
	set(PARAM_DEFAULT_OVERRIDES "{}")
endif()

# get full path for each module
get_property(module_list GLOBAL PROPERTY PX4_MODULE_PATHS)
if (DISABLE_PARAMS_MODULE_SCOPING)
	# search all directories with .c files (potentially containing parameters)
	file(GLOB_RECURSE new_list
		${PX4_SOURCE_DIR}/src/*.c
		${external_module_paths}
		)
	foreach(file_path ${new_list})
		get_filename_component(dir_path ${file_path} PATH)
		list(APPEND module_list "${dir_path}")
	endforeach()
	list(REMOVE_DUPLICATES module_list)
else()
	foreach(module ${config_module_list})
		# include all subdirectories as well
		file(GLOB children RELATIVE ${PX4_SOURCE_DIR}/src/${module} ${PX4_SOURCE_DIR}/src/${module}/*)
		foreach(child ${children})
			if(IS_DIRECTORY ${PX4_SOURCE_DIR}/src/${module}/${child})
				list(APPEND module_list ${PX4_SOURCE_DIR}/src/${module}/${child})
			endif()
		endforeach()

		list(APPEND module_list ${PX4_SOURCE_DIR}/src/${module})
	endforeach()

	list(APPEND module_list ${external_module_paths})
endif()

list(REMOVE_DUPLICATES module_list)

set(parameters_xml ${PX4_BINARY_DIR}/parameters.xml)
file(GLOB_RECURSE param_src_files ${PX4_SOURCE_DIR}/src/*params.c)
add_custom_command(OUTPUT ${parameters_xml}
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/px_process_params.py
		--src-path ${module_list}
		--xml ${parameters_xml}
		--inject-xml ${CMAKE_CURRENT_SOURCE_DIR}/parameters_injected.xml
		--overrides ${PARAM_DEFAULT_OVERRIDES}
		--board ${BOARD}
		#--verbose
	DEPENDS
		${param_src_files}
		parameters_injected.xml
		px4params/srcparser.py
		px4params/srcscanner.py
		px4params/xmlout.py
		px_process_params.py
		parameters_injected.xml
	COMMENT "Generating parameters.xml"
)
add_custom_target(parameters_xml DEPENDS ${parameters_xml})

# generate px4_parameters.c and px4_parameters{,_public}.h
add_custom_command(OUTPUT px4_parameters.c px4_parameters.h px4_parameters_public.h
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/px_generate_params.py
		--xml ${parameters_xml} --dest ${CMAKE_CURRENT_BINARY_DIR}
	DEPENDS
		${PX4_BINARY_DIR}/parameters.xml
		px_generate_params.py
		templates/px4_parameters.c.jinja
		templates/px4_parameters.h.jinja
		templates/px4_parameters_public.h.jinja
	)

set(SRCS)
if ("${CONFIG_SHMEM}" STREQUAL "1")
	message(STATUS "parameters shared memory enabled")
	list(APPEND SRCS param_shmem.c)
else()
	list(APPEND SRCS param.c)
endif()

if(${OS} STREQUAL "nuttx")
	add_subdirectory(flashparams)
endif()

# TODO: find a better way to do this
if (NOT "${CONFIG}" MATCHES "px4io")
	add_library(parameters
		${SRCS}
		px4_parameters.c
		px4_parameters.h
		px4_parameters_public.h
		)
	add_dependencies(parameters prebuild_targets)

	if ("${CONFIG_SHMEM}" STREQUAL "1")
		target_link_libraries(parameters PRIVATE px4_layer)
	endif()

	target_link_libraries(parameters PRIVATE perf tinybson)
	target_compile_definitions(parameters PRIVATE -DMODULE_NAME="parameters")
	target_compile_options(parameters PRIVATE -Wno-sign-compare) # TODO: fix this
else()
	add_library(parameters ${PX4_SOURCE_DIR}/src/platforms/empty.c)
endif()

if(${OS} STREQUAL "nuttx")
	target_link_libraries(parameters PRIVATE flashparams)
endif()
