############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

# actuator metadata generation
get_property(module_config_files GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)
if(config_io_board)
	set(board_with_io_arg --board-with-io)
endif()
set(generated_actuators_metadata_file ${PX4_BINARY_DIR}/actuators.json)
add_custom_command(OUTPUT ${generated_actuators_metadata_file}
	${generated_actuators_metadata_file}.xz
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/module_config/generate_actuators_metadata.py
		${board_with_io_arg}
		--timer-config ${PX4_BOARD_DIR}/src/timer_config.cpp
		--config-files ${module_config_files} #--verbose
		--compress
		--board ${PX4_BOARD}
		--output-file ${generated_actuators_metadata_file}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/validate_json.py
		--schema-file ${PX4_SOURCE_DIR}/src/modules/mavlink/mavlink/component_information/actuators.schema.json
		${generated_actuators_metadata_file}
		--skip-if-no-schema # mavlink submodule might not exist for current target if built in CI
		#--verbose
	DEPENDS
		${module_config_files}
		${PX4_SOURCE_DIR}/Tools/module_config/generate_actuators_metadata.py
		${PX4_SOURCE_DIR}/src/lib/mixer_module/output_functions.yaml
	COMMENT "Generating actuators.json"
)
add_custom_target(actuators_json DEPENDS ${generated_actuators_metadata_file})

