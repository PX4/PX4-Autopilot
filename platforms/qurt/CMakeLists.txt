############################################################################
#
#   Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

px4_qurt_generate_builtin_commands(
        OUT ${PX4_BINARY_DIR}/apps
        MODULE_LIST ${module_libraries}
)

include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
	)

add_library(px4 SHARED
	${PX4_SOURCE_DIR}/platforms/qurt/unresolved_symbols.c
	${PX4_SOURCE_DIR}/platforms/qurt/new_delete.cpp
	${PX4_BINARY_DIR}/apps.cpp
	)

target_link_libraries(px4
	modules__muorb__slpi
	${module_libraries}
	)

# Boards whose shared objects import target runtime symbols can register system
# provider symbol files and prefixes that must always be resolved internally.
get_property(qurt_forbidden_dynamic_import_prefixes GLOBAL
	PROPERTY PX4_QURT_FORBIDDEN_DYNAMIC_IMPORT_PREFIXES)
get_property(qurt_dynamic_import_provider_symbol_files GLOBAL
	PROPERTY PX4_QURT_DYNAMIC_IMPORT_PROVIDER_SYMBOL_FILES)

if(qurt_forbidden_dynamic_import_prefixes
   OR qurt_dynamic_import_provider_symbol_files)
	set(qurt_dynamic_import_args)
	set(qurt_dynamic_import_dependencies
		${PX4_SOURCE_DIR}/Tools/ci/check_elf_imports.py
	)

	foreach(prefix ${qurt_forbidden_dynamic_import_prefixes})
		list(APPEND qurt_dynamic_import_args --forbid-prefix ${prefix})
	endforeach()

	foreach(symbol_file ${qurt_dynamic_import_provider_symbol_files})
		list(APPEND qurt_dynamic_import_args --provider-symbol-file ${symbol_file})
		list(APPEND qurt_dynamic_import_dependencies ${symbol_file})
	endforeach()

	if(qurt_dynamic_import_provider_symbol_files)
		# These shared objects are linked into libpx4.so and supply C++ runtime
		# symbols independently of the target system image.
		set(qurt_dynamic_import_provider_elfs
			${TOOLSLIB}/libc++.so.1
			${TOOLSLIB}/libc++abi.so.1
			${HEXAGON_SDK_ROOT}/libs/weak_refs/ship/hexagon_toolv84/weak_refs.so
		)

		foreach(provider_elf ${qurt_dynamic_import_provider_elfs})
			list(APPEND qurt_dynamic_import_args --provider-elf ${provider_elf})
			list(APPEND qurt_dynamic_import_dependencies ${provider_elf})
		endforeach()
	endif()

	add_custom_target(px4_qurt_check_dynamic_imports ALL
		COMMAND
			${PYTHON_EXECUTABLE}
			${PX4_SOURCE_DIR}/Tools/ci/check_elf_imports.py
			--nm ${HEXAGON_BIN}/${CROSSDEV}nm
			--elf $<TARGET_FILE:px4>
			${qurt_dynamic_import_args}
		DEPENDS
			px4
			${qurt_dynamic_import_dependencies}
		COMMENT "Checking QURT shared-object dynamic imports"
		VERBATIM
	)
endif()
