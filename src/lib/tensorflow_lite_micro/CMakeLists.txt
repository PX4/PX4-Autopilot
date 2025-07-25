############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

if(CONFIG_LIB_TFLM)
	px4_add_git_submodule(TARGET git_tflite-micro PATH tflite_micro)

	set(TFLITE_DOWNLOADS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/tools/make/downloads)

	get_directory_property(FLAGS COMPILE_OPTIONS)
	list(REMOVE_ITEM FLAGS "-Wcast-align")
	set_directory_properties(PROPERTIES COMPILE_OPTIONS "${FLAGS}")


	file(GLOB TFLITE_MICRO_SRCS
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/arena_allocator/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/memory_planner/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/tflite_bridge/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/kernels/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/internal/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/internal/reference/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/core/api/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/core/c/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/schema/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/compiler/mlir/lite/core/api/*.cc
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/compiler/mlir/lite/schema/*.cc
	)
	# Filter out tests as they cause errors
	list(FILTER TFLITE_MICRO_SRCS EXCLUDE REGEX ".*_test.*\\.cc$")


	set(TFLM_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro
	${TFLITE_DOWNLOADS_DIR}
	${TFLITE_DOWNLOADS_DIR}/flatbuffers/include
	${TFLITE_DOWNLOADS_DIR}/ruy
	${TFLITE_DOWNLOADS_DIR}/gemmlowp
	${TFLITE_DOWNLOADS_DIR}/cmsis/Cortex_DFP/Device/ARMCM7/Include
	${TFLITE_DOWNLOADS_DIR}/cmsis/CMSIS/Core/Include
	${TFLITE_DOWNLOADS_DIR}/cmsis
	)

	set(TFLM_BUILD_TIMESTAMP ${CMAKE_CURRENT_BINARY_DIR}/tflm_build_complete.timestamp)
	add_custom_command(
		OUTPUT ${TFLM_BUILD_TIMESTAMP}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			${CMAKE_CURRENT_SOURCE_DIR}/generate_cc_arrays.py
			${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/tools/generate_cc_arrays.py
		# TODO maybe change this if building for other architectures
		COMMAND make -f tensorflow/lite/micro/tools/make/Makefile MICRO_LITE_EXAMPLE_TESTS= MICRO_LITE_BENCHMARKS= MICRO_LITE_TEST_SRCS= MICRO_LITE_INTEGRATION_TESTS= third_party_downloads
		# Create timestamp file to mark completion
		COMMAND ${CMAKE_COMMAND} -E touch ${TFLM_BUILD_TIMESTAMP}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro
		COMMENT "Downloading TFLM third party dependencies"
		DEPENDS git_tflite-micro
		VERBATIM
	)
	add_custom_target(build_tflm_native
		DEPENDS ${TFLM_BUILD_TIMESTAMP}
	)

	px4_add_library(tensorflow_lite_micro ${TFLITE_MICRO_SRCS})

	if(CONFIG_BOARD_TOOLCHAIN STREQUAL "arm-none-eabi")
		add_definitions(-include "${CMAKE_CURRENT_SOURCE_DIR}/include/px4_tflm_fix.h")
	endif()

	target_include_directories(tensorflow_lite_micro PUBLIC ${TFLM_INCLUDE_DIRS})
	add_dependencies(tensorflow_lite_micro build_tflm_native)
	target_compile_features(tensorflow_lite_micro PRIVATE cxx_std_17)

	target_compile_options(tensorflow_lite_micro PUBLIC
	-Wno-float-equal
	-Wno-shadow
	)


endif()
