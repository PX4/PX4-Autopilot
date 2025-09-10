############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#
# Vision Target Estimator (VTEST) Symforce code generation
#
option(VTEST_SYMFORCE_GEN "vtest generate symforce output" OFF)
option(VTEST_UPDATE_COMMITTED_DERIVATION "Rewrite committed default generated files" OFF)

# Check if symforce is available
execute_process(
	COMMAND ${PYTHON_EXECUTABLE} -m symforce.symbolic
	RESULT_VARIABLE PYTHON_SYMFORCE_EXIT_CODE
	OUTPUT_QUIET
	ERROR_QUIET
)

# Enable symforce generation if a non-default configuration is used
if(CONFIG_VTEST_MOVING)
	set(VTEST_SYMFORCE_GEN ON)
endif()

# Set up paths and default variables
set(VTEST_DERIVATION_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Position/vtest_derivation)

set(VTEST_GENERATED_FILE_LIST
	${VTEST_DERIVATION_SRC_DIR}/generated/state.h
	${VTEST_DERIVATION_SRC_DIR}/generated/predictCov.h
	${VTEST_DERIVATION_SRC_DIR}/generated/computeInnovCov.h
	${VTEST_DERIVATION_SRC_DIR}/generated/predictState.h
	${VTEST_DERIVATION_SRC_DIR}/generated/syncState.h
)

# Use one file as the dependency anchor. The custom_command OUTPUT will ensure all are generated.
set(VTEST_GENERATED_FILES ${VTEST_DERIVATION_SRC_DIR}/generated/state.h)
set(VTEST_GENERATED_DERIVATION_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Position)

if(VTEST_SYMFORCE_GEN AND (${PYTHON_SYMFORCE_EXIT_CODE} EQUAL 0))

	set(VTEST_SYMFORCE_TARGET_DEPS)

	# 1. Regenerate default committed files in the source tree
	# This can be disabled to prevent accidental changes to committed files.
	if(VTEST_UPDATE_COMMITTED_DERIVATION)
		add_custom_command(
			OUTPUT
				${VTEST_GENERATED_FILE_LIST}
			COMMAND
				${PYTHON_EXECUTABLE} ${VTEST_DERIVATION_SRC_DIR}/derivation.py
			DEPENDS
				${VTEST_DERIVATION_SRC_DIR}/derivation.py
				${VTEST_DERIVATION_SRC_DIR}/utils/derivation_utils.py

			WORKING_DIRECTORY ${VTEST_DERIVATION_SRC_DIR}
			COMMENT "VTEST Symforce code generation (default, committed)"
			USES_TERMINAL
		)
		list(APPEND VTEST_SYMFORCE_TARGET_DEPS ${VTEST_GENERATED_FILE_LIST})
	endif()

	# 2. Generate files to the build directory for non-default configurations
	set(VTEST_DERIVATION_DST_DIR ${CMAKE_CURRENT_BINARY_DIR}/vtest_derivation)
	file(MAKE_DIRECTORY ${VTEST_DERIVATION_DST_DIR})

	set(VTEST_GENERATED_FILE_LIST_DST
		${VTEST_DERIVATION_DST_DIR}/generated/state.h
		${VTEST_DERIVATION_DST_DIR}/generated/predictCov.h
		${VTEST_DERIVATION_DST_DIR}/generated/computeInnovCov.h
		${VTEST_DERIVATION_DST_DIR}/generated/predictState.h
		${VTEST_DERIVATION_DST_DIR}/generated/syncState.h
	)

	# Overwrite variables to use build directory paths for the compile step
	set(VTEST_GENERATED_FILES ${VTEST_DERIVATION_DST_DIR}/generated/state.h)
	set(VTEST_GENERATED_DERIVATION_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR}/..)

	set(SYMFORCE_ARGS)
	if(CONFIG_VTEST_MOVING)
		message(STATUS "vtest: symforce --moving")
		list(APPEND SYMFORCE_ARGS --moving)
	endif()

	add_custom_command(
		OUTPUT
			${VTEST_GENERATED_FILE_LIST_DST}
		COMMAND
			${PYTHON_EXECUTABLE} ${VTEST_DERIVATION_SRC_DIR}/derivation.py ${SYMFORCE_ARGS}
		DEPENDS
			${VTEST_DERIVATION_SRC_DIR}/derivation.py
			${VTEST_DERIVATION_SRC_DIR}/utils/derivation_utils.py

		WORKING_DIRECTORY ${VTEST_DERIVATION_DST_DIR}
		COMMENT "VTEST Symforce code generation"
		USES_TERMINAL
	)
	list(APPEND VTEST_SYMFORCE_TARGET_DEPS ${VTEST_GENERATED_FILE_LIST_DST})

	add_custom_target(vtest_symforce_generate
		DEPENDS ${VTEST_SYMFORCE_TARGET_DEPS}
	)
endif()

px4_add_module(
	MODULE modules__vision_target_estimator
	MAIN vision_target_estimator
	COMPILE_FLAGS
	STACK_MAX
		5000
	INCLUDES
		${VTEST_GENERATED_DERIVATION_INCLUDE_PATH}
	SRCS
		VisionTargetEst.cpp
		Position/VTEPosition.cpp
		Position/KF_position.cpp
		Orientation/VTEOrientation.cpp
		Orientation/KF_orientation.cpp

		${VTEST_GENERATED_FILES}
	DEPENDS
		mathlib
		px4_work_queue
)
