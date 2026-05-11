############################################################################
#
# Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#=============================================================================
#
#	px4_add_common_flags
#
#	Set the default build flags.
#
#	Usage:
#		px4_add_common_flags()
#

function(px4_msvc_compile_options_from_gnu out_var)
	set(msvc_flags)

	# Keep GNU-like warning names as the common policy vocabulary and translate
	# the MSVC equivalents here when the mapping is exact enough to be useful.
	foreach(flag IN LISTS ARGN)
		if("${flag}" STREQUAL "-Werror")
			list(APPEND msvc_flags /WX)
		elseif("${flag}" STREQUAL "-Wno-error")
			list(APPEND msvc_flags /WX-)
		elseif("${flag}" STREQUAL "-Wall" OR "${flag}" STREQUAL "-Wextra")
			list(APPEND msvc_flags /W4)
		elseif("${flag}" STREQUAL "-Wno-array-bounds")
			list(APPEND msvc_flags /wd4789)
		elseif("${flag}" STREQUAL "-Wno-deprecated-copy")
			list(APPEND msvc_flags /wd5267)
		elseif("${flag}" STREQUAL "-Wno-deprecated-declarations")
			list(APPEND msvc_flags /wd4996)
		elseif("${flag}" STREQUAL "-Wno-empty-body")
			list(APPEND msvc_flags /wd4390)
		elseif("${flag}" STREQUAL "-Wno-implicit-function-declaration")
			list(APPEND msvc_flags /wd4013)
		elseif("${flag}" STREQUAL "-Wno-implicit-fallthrough")
			list(APPEND msvc_flags /wd5262)
		elseif("${flag}" STREQUAL "-Wno-incompatible-pointer-types")
			list(APPEND msvc_flags /wd4133)
		elseif("${flag}" STREQUAL "-Wno-maybe-uninitialized" OR "${flag}" STREQUAL "-Wno-uninitialized")
			list(APPEND msvc_flags /wd4701 /wd4703)
		elseif("${flag}" STREQUAL "-Wno-overloaded-virtual")
			list(APPEND msvc_flags /wd4263 /wd4264)
		elseif("${flag}" STREQUAL "-Wno-shadow")
			list(APPEND msvc_flags /wd4456 /wd4457 /wd4458 /wd4459)
		elseif("${flag}" STREQUAL "-Wno-sign-compare")
			list(APPEND msvc_flags /wd4018 /wd4389)
		elseif("${flag}" STREQUAL "-Wno-strict-prototypes")
			list(APPEND msvc_flags /wd4255)
		elseif("${flag}" STREQUAL "-Wno-switch")
			list(APPEND msvc_flags /wd4061 /wd4062)
		elseif("${flag}" STREQUAL "-Wno-type-limits")
			list(APPEND msvc_flags /wd4296)
		elseif("${flag}" STREQUAL "-Wno-unknown-pragmas")
			list(APPEND msvc_flags /wd4068)
		elseif("${flag}" STREQUAL "-Wno-unused")
			list(APPEND msvc_flags /wd4100 /wd4101 /wd4189 /wd4505)
		elseif("${flag}" STREQUAL "-Wno-unused-but-set-parameter")
			list(APPEND msvc_flags /wd4100)
		elseif("${flag}" STREQUAL "-Wno-unused-but-set-variable")
			list(APPEND msvc_flags /wd4189)
		elseif("${flag}" STREQUAL "-Wno-unused-function")
			list(APPEND msvc_flags /wd4505)
		elseif("${flag}" STREQUAL "-Wno-unused-parameter")
			list(APPEND msvc_flags /wd4100)
		elseif("${flag}" STREQUAL "-Wno-unused-variable")
			list(APPEND msvc_flags /wd4101 /wd4189)
		elseif("${flag}" STREQUAL "-Wno-write-strings")
			list(APPEND msvc_flags /wd4090)
		endif()
	endforeach()

	if(msvc_flags)
		list(REMOVE_DUPLICATES msvc_flags)
	endif()

	set(${out_var} ${msvc_flags} PARENT_SCOPE)
endfunction()

function(px4_add_compile_options_for_compiler)
	cmake_parse_arguments(arg "" "" "GNU_LIKE;MSVC" ${ARGN})
	set(is_gnu_like_compiler FALSE)

	if((CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang"))
		set(is_gnu_like_compiler TRUE)
	endif()

	if(MSVC)
		if(arg_MSVC)
			add_compile_options(${arg_MSVC})
		else()
			px4_msvc_compile_options_from_gnu(msvc_flags ${arg_GNU_LIKE})

			if(msvc_flags)
				add_compile_options(${msvc_flags})
			endif()
		endif()
	elseif(is_gnu_like_compiler AND arg_GNU_LIKE)
		add_compile_options(${arg_GNU_LIKE})
	endif()
endfunction()

function(px4_add_c_compile_options_for_compiler)
	cmake_parse_arguments(arg "" "" "GNU_LIKE;MSVC" ${ARGN})

	if(MSVC)
		if(arg_MSVC)
			set(msvc_flags ${arg_MSVC})
		else()
			px4_msvc_compile_options_from_gnu(msvc_flags ${arg_GNU_LIKE})
		endif()

		foreach(flag ${msvc_flags})
			add_compile_options($<$<COMPILE_LANGUAGE:C>:${flag}>)
		endforeach()
	elseif(arg_GNU_LIKE AND (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang"))
		foreach(flag ${arg_GNU_LIKE})
			add_compile_options($<$<COMPILE_LANGUAGE:C>:${flag}>)
		endforeach()
	endif()
endfunction()

function(px4_target_compile_options_for_compiler target scope)
	cmake_parse_arguments(arg "" "" "GNU_LIKE;MSVC" ${ARGN})
	set(is_gnu_like_compiler FALSE)

	if((CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang"))
		set(is_gnu_like_compiler TRUE)
	endif()

	if(MSVC)
		if(arg_MSVC)
			target_compile_options(${target} ${scope} ${arg_MSVC})
		else()
			px4_msvc_compile_options_from_gnu(msvc_flags ${arg_GNU_LIKE})

			if(msvc_flags)
				target_compile_options(${target} ${scope} ${msvc_flags})
			endif()
		endif()
	elseif(is_gnu_like_compiler AND arg_GNU_LIKE)
		target_compile_options(${target} ${scope} ${arg_GNU_LIKE})
	endif()
endfunction()

function(px4_target_c_compile_options_for_compiler target scope)
	cmake_parse_arguments(arg "" "" "GNU_LIKE;MSVC" ${ARGN})

	if(MSVC)
		if(arg_MSVC)
			set(msvc_flags ${arg_MSVC})
		else()
			px4_msvc_compile_options_from_gnu(msvc_flags ${arg_GNU_LIKE})
		endif()

		foreach(flag ${msvc_flags})
			target_compile_options(${target} ${scope} $<$<COMPILE_LANGUAGE:C>:${flag}>)
		endforeach()
	elseif(arg_GNU_LIKE AND (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang|AppleClang"))
		foreach(flag ${arg_GNU_LIKE})
			target_compile_options(${target} ${scope} $<$<COMPILE_LANGUAGE:C>:${flag}>)
		endforeach()
	endif()
endfunction()

function(px4_add_common_flags)

	# clang-cl emits CMAKE_CXX_COMPILER_ID="Clang" with
	# CMAKE_CXX_SIMULATE_ID="MSVC" because it is the LLVM driver running
	# on top of the MSVC headers/CRT. Treat it like native MSVC for flag
	# selection so we don't try to forward GCC-style switches like
	# -fdata-sections that the cl-style driver rejects.
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" OR
	    (("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") AND ("${CMAKE_CXX_SIMULATE_ID}" STREQUAL "MSVC")))
		add_compile_options(
			/MP
			/Zi
			/EHsc
			/bigobj
			/permissive-
			/Zc:preprocessor
			/utf-8
			/FIvisibility.h
			/W3

			/wd4005 # macro redefinition in Windows/POSIX compatibility headers
			/wd4068 # unknown pragma from GCC-oriented third-party code
			/wd4244 # narrowing conversion warnings are pervasive in PX4 SITL
			/wd4267 # size_t to int narrowing in POSIX-style APIs
			/wd4305 # truncation from double to float constants
			/wd4996 # POSIX/CRT compatibility names are intentional
		)
	else()
		add_compile_options(
			-g # always build debug symbols

			# optimization options
			-fdata-sections
			-ffunction-sections
			-fomit-frame-pointer
			-fmerge-all-constants

			#-funsafe-math-optimizations # Enables -fno-signed-zeros, -fno-trapping-math, -fassociative-math and -freciprocal-math
			-fno-signed-zeros	# Allow optimizations for floating-point arithmetic that ignore the signedness of zero
			-fno-trapping-math	# Compile code assuming that floating-point operations cannot generate user-visible traps
			#-fassociative-math	# Allow re-association of operands in series of floating-point operations
			-freciprocal-math	# Allow the reciprocal of a value to be used instead of dividing by the value if this enables optimizations

			-fno-math-errno		# Do not set errno after calling math functions that are executed with a single instruction, e.g., sqrt

			-fno-strict-aliasing

			# visibility
			-fvisibility=hidden
			-include visibility.h

			# Warnings
			-Wall
			-Wextra
			-Werror

			-Warray-bounds
			-Wcast-align
			-Wdisabled-optimization
			-Wdouble-promotion
			-Wfatal-errors
			-Wfloat-equal
			-Wformat-security
			-Winit-self
			-Wlogical-op
			-Wpointer-arith
			-Wshadow
			-Wuninitialized
			-Wunknown-pragmas
			-Wunused-variable

			# disabled warnings
			-Wno-missing-field-initializers
			-Wno-missing-include-dirs # TODO: fix and enable
			-Wno-unused-parameter

			)
	endif()

	# compiler specific flags
	if (("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang"))

		add_compile_options(
			-fcolor-diagnostics # force color for clang (needed for clang + ccache)
			-fdiagnostics-absolute-paths # force absolute paths

			-Qunused-arguments

			-Wno-c99-designator
			-Wno-unknown-warning-option
			-Wno-unused-const-variable
			-Wno-varargs
			-Wno-vla-cxx-extension # FIXME: do not use variable length arrays
		)

	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
			# force color for gcc > 4.9
			add_compile_options(-fdiagnostics-color=always)
		endif()

		if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.3)
			add_compile_options(-Wno-stringop-truncation)
		endif()

		add_compile_options(
			-fno-builtin-printf
			-fno-strength-reduce

			-Wformat=1
			-Wunused-but-set-variable

			-Wno-format-truncation # TODO: fix
		)

		# -fcheck-new is a no-op for Clang in general
		# and has no effect, but can generate a compile
		# error for some OS
		add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fcheck-new>)

	elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
		message(FATAL_ERROR "Intel compiler not yet supported")
	endif()

	# C only flags
	set(c_flags)
	if (NOT ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC"))
		list(APPEND c_flags
			-fno-common

			-Wnested-externs
			-Wstrict-prototypes
		)
	endif()
	foreach(flag ${c_flags})
		add_compile_options($<$<COMPILE_LANGUAGE:C>:${flag}>)
	endforeach()


	# CXX only flags
	set(cxx_flags)
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		list(APPEND cxx_flags
			/Zc:__cplusplus
		)
	else()
		list(APPEND cxx_flags
			-Wreorder

			# disabled warnings
			-Wno-overloaded-virtual # TODO: fix and remove
		)
	endif()

	if((NOT BUILD_TESTING) AND (NOT PX4_CONFIG MATCHES "px4_sitl") AND (NOT ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")))
		list(APPEND cxx_flags
			-fno-rtti
		)
	endif()

	foreach(flag ${cxx_flags})
		add_compile_options($<$<COMPILE_LANGUAGE:CXX>:${flag}>)
	endforeach()


	include_directories(
		${PX4_BINARY_DIR}
		${PX4_BINARY_DIR}/src/lib

		${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/src/px4/${PX4_CHIP_MANUFACTURER}/${PX4_CHIP}/include
		${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/src/px4/common/include
		${PX4_SOURCE_DIR}/platforms/common
		${PX4_SOURCE_DIR}/platforms/common/include

		${PX4_SOURCE_DIR}/src
		${PX4_SOURCE_DIR}/src/include
		${PX4_SOURCE_DIR}/src/lib
		${PX4_SOURCE_DIR}/src/lib/matrix
		${PX4_SOURCE_DIR}/src/modules
	)
	if(EXISTS ${PX4_BOARD_DIR}/include)
		include_directories(${PX4_BOARD_DIR}/include)
	endif()

	add_definitions(
		-DCONFIG_ARCH_BOARD_${PX4_BOARD_NAME}
		-D__CUSTOM_FILE_IO__
		-D__STDC_FORMAT_MACROS
		)

endfunction()
