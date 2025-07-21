############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

# Metadata - helpers for generating documentation

add_custom_target(metadata_airframes
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/docs
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		-v -a ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d
		--markdown ${PX4_BINARY_DIR}/docs/airframes.md
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		-v -a ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d
		--xml ${PX4_BINARY_DIR}/docs/airframes.xml
	COMMENT "Generating full airframe metadata (markdown and xml)"
	USES_TERMINAL
)


set(generated_params_dir ${PX4_BINARY_DIR}/generated_params_metadata)
file(GLOB_RECURSE yaml_config_files ${PX4_SOURCE_DIR}/src/modules/*.yaml
	${PX4_SOURCE_DIR}/src/drivers/*.yaml ${PX4_SOURCE_DIR}/src/lib/*.yaml)

# avoid param duplicates
list(FILTER yaml_config_files EXCLUDE REGEX ".*/pwm_out_sim/")
list(FILTER yaml_config_files EXCLUDE REGEX ".*/linux_pwm_out/")
list(FILTER yaml_config_files EXCLUDE REGEX ".*/spacecraft/")

add_custom_target(metadata_parameters
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/docs
	COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_params_dir}

	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/serial/generate_config.py
		--all-ports --ethernet --params-file ${generated_params_dir}/serial_params.c --config-files ${yaml_config_files}

	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/module_config/generate_params.py
		--params-file ${generated_params_dir}/module_params.c
		--timer-config ${PX4_SOURCE_DIR}/boards/px4/fmu-v5/src/timer_config.cpp # select a typical board
		--board-with-io
		--ethernet
		--config-files ${yaml_config_files} #--verbose

	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/parameters/px_process_params.py
		--src-path `find ${PX4_SOURCE_DIR}/src -maxdepth 4 -type d` ${generated_params_dir}
		--inject-xml ${PX4_SOURCE_DIR}/src/lib/parameters/parameters_injected.xml
		--markdown ${PX4_BINARY_DIR}/docs/parameters.md

	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/parameters/px_process_params.py
		--src-path `find ${PX4_SOURCE_DIR}/src -maxdepth 4 -type d` ${generated_params_dir}
		--inject-xml ${PX4_SOURCE_DIR}/src/lib/parameters/parameters_injected.xml
		--json ${PX4_BINARY_DIR}/docs/parameters.json
		--compress

	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/parameters/px_process_params.py
		--src-path `find ${PX4_SOURCE_DIR}/src -maxdepth 4 -type d` ${generated_params_dir}
		--inject-xml ${PX4_SOURCE_DIR}/src/lib/parameters/parameters_injected.xml
		--xml ${PX4_BINARY_DIR}/docs/parameters.xml

	COMMENT "Generating full parameter metadata (markdown, xml, and json)"
	USES_TERMINAL
)

add_custom_target(metadata_module_documentation
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/docs
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_module_doc.py -v --src-path ${PX4_SOURCE_DIR}/src
		--markdown ${PX4_BINARY_DIR}/docs/modules
	COMMENT "Generating module documentation"
	USES_TERMINAL
)

set(events_src_path "${PX4_SOURCE_DIR}/src/lib/events")
add_custom_target(metadata_extract_events
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/events
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_events.py
		--src-path ${PX4_SOURCE_DIR}/src
		--json ${PX4_BINARY_DIR}/events/px4_full.json #--verbose
	COMMAND ${PYTHON_EXECUTABLE} ${events_src_path}/libevents/scripts/combine.py
		${PX4_BINARY_DIR}/events/px4_full.json
		${events_src_path}/libevents/events/common.json
		${events_src_path}/enums.json
		--output ${PX4_BINARY_DIR}/events/all_events_full.json
	COMMAND ${PYTHON_EXECUTABLE} ${events_src_path}/libevents/scripts/validate.py
		${PX4_BINARY_DIR}/events/all_events_full.json
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/compress.py
		${PX4_BINARY_DIR}/events/all_events_full.json
	COMMENT "Extracting events from full source"
	USES_TERMINAL
)

add_custom_target(all_metadata
	DEPENDS
		metadata_airframes
		metadata_parameters
		metadata_module_documentation
		metadata_extract_events
)
