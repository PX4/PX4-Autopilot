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

if(NOT DEFINED ZEPHYR_BASE)
	message(FATAL_ERROR "Zephyr targets are built with west, see platforms/zephyr/app")
endif()

function(px4_os_add_flags)

	add_definitions(
		-D__PX4_POSIX
		-D__PX4_ZEPHYR
		-Dnoreturn_function=__attribute__\(\(noreturn\)\)
		)

	include_directories(
		platforms/zephyr/include
		platforms/posix/include
		platforms/posix/src/px4/common/include
		platforms/posix/src/px4/generic/generic/include
	)

	# the Zephyr compile options come before the options of each target, so that MAX_CUSTOM_OPT_LEVEL
	# builds the estimators and controllers with -O2 instead of the -Os of Zephyr, as on NuttX
	add_compile_options($<TARGET_PROPERTY:zephyr_interface,INTERFACE_COMPILE_OPTIONS>)

	link_libraries(px4_zephyr_interface)

endfunction()

function(px4_os_determine_build_chip)

	set(PX4_CHIP "generic" CACHE STRING "PX4 Chip" FORCE)
	set(PX4_CHIP_MANUFACTURER "generic" CACHE STRING "PX4 Chip Manufacturer" FORCE)

endfunction()

# arguments are not parsed, Zephyr does not allow setting BOARD
function(px4_os_prebuild_targets)

	# Zephyr compile settings, with the Zephyr headers treated as system headers
	add_library(px4_zephyr_interface INTERFACE)
	target_include_directories(px4_zephyr_interface SYSTEM INTERFACE
		$<TARGET_PROPERTY:zephyr_interface,INTERFACE_INCLUDE_DIRECTORIES>
		$<TARGET_PROPERTY:zephyr_interface,INTERFACE_SYSTEM_INCLUDE_DIRECTORIES>
	)
	target_compile_definitions(px4_zephyr_interface INTERFACE
		$<TARGET_PROPERTY:zephyr_interface,INTERFACE_COMPILE_DEFINITIONS>
	)
	target_compile_options(px4_zephyr_interface INTERFACE
		# PX4 uses GNU language extensions, as on the other platforms
		$<$<COMPILE_LANGUAGE:C>:-std=gnu11>
		$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>
	)
	add_dependencies(px4_zephyr_interface zephyr_generated_headers)

	add_library(prebuild_targets INTERFACE)
	target_link_libraries(prebuild_targets INTERFACE px4_layer drivers_board px4_zephyr_interface)
	add_dependencies(prebuild_targets DEPENDS uorb_headers)

endfunction()
