############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

# SBOM - SPDX 2.3 JSON Software Bill of Materials generation

option(GENERATE_SBOM "Generate SPDX 2.3 SBOM" ON)

if(DEFINED ENV{PX4_SBOM_DISABLE})
	set(GENERATE_SBOM OFF)
endif()

if(GENERATE_SBOM)

	# Write board-specific module list for the SBOM generator
	set(sbom_module_list_file "${PX4_BINARY_DIR}/config_module_list.txt")
	get_property(module_list GLOBAL PROPERTY PX4_MODULE_PATHS)
	string(REPLACE ";" "\n" module_list_content "${module_list}")
	file(GENERATE OUTPUT ${sbom_module_list_file} CONTENT "${module_list_content}\n")

	set(sbom_output "${PX4_BINARY_DIR}/${PX4_CONFIG}.sbom.spdx.json")

	add_custom_command(
		OUTPUT ${sbom_output}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/ci/generate_sbom.py
			--source-dir ${PX4_SOURCE_DIR}
			--board ${PX4_CONFIG}
			--modules-file ${sbom_module_list_file}
			--compiler ${CMAKE_C_COMPILER}
			--platform ${PX4_PLATFORM}
			--output ${sbom_output}
		DEPENDS
			${PX4_SOURCE_DIR}/Tools/ci/generate_sbom.py
			${PX4_SOURCE_DIR}/Tools/ci/license-overrides.yaml
			${PX4_SOURCE_DIR}/.gitmodules
			${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
			${sbom_module_list_file}
		COMMENT "Generating SPDX SBOM for ${PX4_CONFIG}"
	)

	add_custom_target(sbom ALL DEPENDS ${sbom_output})

endif()
