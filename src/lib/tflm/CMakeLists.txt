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
	# message(STATUS "Implicit C++ include dirs: ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}")

	px4_add_git_submodule(TARGET git_tflite-micro PATH tflite_micro)

	set(TFLITE_DOWNLOADS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/px4)

	get_directory_property(FLAGS COMPILE_OPTIONS)
	list(REMOVE_ITEM FLAGS "-Wcast-align")
	set_directory_properties(PROPERTIES COMPILE_OPTIONS "${FLAGS}")

	#set(TFLITE_MICRO_SRCS)

	# macro(add_directory_sources dir)
	# 	if (EXISTS "${dir}")
	# 		file(GLOB _sources "${dir}/*.cc")
	# 		list(FILTER _sources EXCLUDE REGEX ".*_test.*\\.cc$")
	# 		list(APPEND TFLITE_MICRO_SRCS ${_sources})
	# 	endif()
	# endmacro()

	# # Add sources from each directory
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/arena_allocator)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/memory_planner)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/tflite_bridge)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/micro/kernels)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/internal)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/kernels/internal/reference)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/core/api)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/core/c)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/lite/schema)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/compiler/mlir/lite/core/api)
	# add_directory_sources(${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/tensorflow/compiler/mlir/lite/schema)


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
	${TFLITE_DOWNLOADS_DIR}/ruy
	${TFLITE_DOWNLOADS_DIR}/gemmlowp
	${TFLITE_DOWNLOADS_DIR}/cmsis/Cortex_DFP/Device/ARMCM7/Include
	${TFLITE_DOWNLOADS_DIR}/cmsis/CMSIS/Core/Include
	${TFLITE_DOWNLOADS_DIR}/cmsis
	)

	px4_add_library(tflm ${TFLITE_MICRO_SRCS})

	# Add C++ 17 std lib if building for NuttX
	# if(CONFIG_BOARD_TOOLCHAIN STREQUAL "arm-none-eabi")
	# 	list(APPEND TFLM_INCLUDE_DIRS
	# 		${TFLITE_DOWNLOADS_DIR}/include/13.2.1
	# 		${TFLITE_DOWNLOADS_DIR}/include/13.2.1/arm-none-eabi
	# 	)
#   set_target_properties(tflm PROPERTIES
#   COMPILE_FLAGS "-include ${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro/px4/compiler_fix.h"
# )
	# endif()
	# bring in the compiler’s standard C++ headers so <functional> etc. resolve:
	get_target_property(_opts tflm COMPILE_OPTIONS)
	list(FILTER _opts EXCLUDE REGEX "-nostdinc\\+\\+")
	set_target_properties(tflm PROPERTIES COMPILE_OPTIONS "${_opts}")

	target_include_directories(tflm PUBLIC ${TFLM_INCLUDE_DIRS})



	# if(CONFIG_BOARD_TOOLCHAIN STREQUAL "arm-none-eabi")
	# 	get_directory_property(FLAGS COMPILE_OPTIONS)
	# 	list(REMOVE_ITEM FLAGS "-nostdinc++")
	# 	set_directory_properties(PROPERTIES COMPILE_OPTIONS "${FLAGS}")
	# 	foreach(dir IN LISTS CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES)
	# 		target_compile_options(tflm PRIVATE "-isystem" "${dir}")
	# 	endforeach()
	# endif()
	if(CONFIG_BOARD_TOOLCHAIN STREQUAL "arm-none-eabi")
		get_target_property(_inc_dirs tflm INCLUDE_DIRECTORIES)
		# drop any “…/platforms/nuttx/NuttX/include/cxx” entries
		list(FILTER _inc_dirs EXCLUDE REGEX ".*/platforms/nuttx/NuttX/include/cxx$")
		set_target_properties(tflm PROPERTIES INCLUDE_DIRECTORIES "${_inc_dirs}")
		# re‑add the toolchain’s C++ include dirs as -isystem AFTER -nostdinc++
		target_compile_options(tflm PRIVATE
			-isystem${CMAKE_CURRENT_SOURCE_DIR}/tflite_micro
		)
		foreach(dir IN LISTS CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES)
			# this single generator expression produces one flag: -isystem/path
			target_compile_options(tflm PRIVATE
			$<$<COMPILE_LANGUAGE:CXX>:-isystem${dir}>
		)
		endforeach()
	endif()


	target_compile_options(tflm PUBLIC
	-Wno-float-equal
	-Wno-shadow
	-fpermissive
	)


endif()
