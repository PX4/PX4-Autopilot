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

include(cygwin_cygpath)

set(NUTTX_DIR ${PX4_BINARY_DIR}/NuttX/nuttx)
set(NUTTX_APPS_DIR ${PX4_BINARY_DIR}/NuttX/apps)

add_subdirectory(src)

add_executable(px4 ${PX4_SOURCE_DIR}/src/platforms/empty.c)
set(FW_NAME ${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}.elf)
set_target_properties(px4 PROPERTIES OUTPUT_NAME ${FW_NAME})
add_dependencies(px4 git_nuttx nuttx_build)

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

# build NuttX
add_subdirectory(NuttX ${PX4_BINARY_DIR}/NuttX)

set(nuttx_libs)
list(APPEND nuttx_libs
	nuttx_apps
	nuttx_arch
	nuttx_binfmt
	nuttx_c
	nuttx_configs
	nuttx_xx
	nuttx_drivers
	nuttx_fs
	nuttx_mm
	nuttx_sched
	)

if (CONFIG_NET)
	list(APPEND nuttx_libs nuttx_net)
	target_link_libraries(nuttx_fs INTERFACE nuttx_net)
endif()

file(RELATIVE_PATH PX4_BINARY_DIR_REL ${CMAKE_CURRENT_BINARY_DIR} ${PX4_BINARY_DIR})

# only in the cygwin environment: convert absolute linker script path to mixed windows (C:/...)
# because even relative linker script paths are different for linux, mac and windows
CYGPATH(PX4_BINARY_DIR PX4_BINARY_DIR_CYG)

target_link_libraries(nuttx_arch
	INTERFACE
		drivers_board
		drivers_arch
		drivers_boards_common
		drivers_boards_common_arch
)

target_link_libraries(nuttx_c INTERFACE nuttx_drivers)
target_link_libraries(nuttx_xx INTERFACE nuttx_c)

target_link_libraries(px4 PRIVATE

	-nostartfiles
	-nodefaultlibs
	-nostdlib
	-nostdinc++

	-fno-exceptions
	-fno-rtti

	-Wl,--script=${PX4_BINARY_DIR_CYG}/NuttX/nuttx-config/scripts/script.ld
	-Wl,-Map=${PX4_CONFIG}.map
	-Wl,--warn-common
	-Wl,--gc-sections

	-Wl,--start-group
		drivers_boards_common_arch
		${nuttx_libs}
	-Wl,--end-group

	m
	gcc
	)

target_link_libraries(px4 PRIVATE ${module_libraries})

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	target_link_libraries(px4 PRIVATE romfs)
endif()

add_custom_command(OUTPUT ${PX4_BINARY_DIR_REL}/${PX4_BOARD}.bin
	COMMAND ${CMAKE_OBJCOPY} -O binary ${PX4_BINARY_DIR_REL}/${FW_NAME} ${PX4_BINARY_DIR_REL}/${PX4_BOARD}.bin
	DEPENDS px4
	)

# create .px4 with parameter and airframe metadata
if (TARGET parameters_xml AND TARGET airframes_xml)

	string(REPLACE ".elf" ".px4" fw_package ${PX4_BINARY_DIR}/${FW_NAME})

	add_custom_command(
		OUTPUT ${fw_package}
		COMMAND
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
				--prototype ${PX4_SOURCE_DIR}/boards/${PX4_BOARD_VENDOR}/${PX4_BOARD_MODEL}/firmware.prototype
				--git_identity ${PX4_SOURCE_DIR}
				--parameter_xml ${PX4_BINARY_DIR}/parameters.xml
				--airframe_xml ${PX4_BINARY_DIR}/airframes.xml
				--image ${PX4_BINARY_DIR}/${PX4_BOARD}.bin > ${fw_package}
		DEPENDS
			${PX4_BINARY_DIR}/${PX4_BOARD}.bin
			airframes_xml
			parameters_xml
		COMMENT "Creating ${fw_package}"
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		)

	add_custom_target(px4_package ALL DEPENDS ${fw_package})

	# upload helper
	# create upload target helper if NuttX USB CDCACM is present
	if (CONFIG_CDCACM)

		# NuttX CDCACM vendor and product strings
		set(vendorstr_underscore)
		set(productstr_underscore)
		string(REPLACE " " "_" vendorstr_underscore ${CONFIG_CDCACM_VENDORSTR})
		string(REPLACE " " "_" productstr_underscore ${CONFIG_CDCACM_PRODUCTSTR})

		set(serial_ports)
		if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")

			set(px4_usb_path "${vendorstr_underscore}_${productstr_underscore}")

			list(APPEND serial_ports
				# NuttX vendor + product string
				/dev/serial/by-id/*-${px4_usb_path}*

				# Bootloader
				/dev/serial/by-id/*PX4_BL* # typical bootloader USB device string
				/dev/serial/by-id/*BL_FMU*

				# TODO: handle these per board
				/dev/serial/by-id/usb-The_Autopilot*
				/dev/serial/by-id/usb-Bitcraze*
				/dev/serial/by-id/pci-Bitcraze*
				/dev/serial/by-id/usb-Gumstix*

				)

		elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin")
			list(APPEND serial_ports
				/dev/tty.usbmodemPX*,/dev/tty.usbmodem*
				)
		elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "CYGWIN")
			list(APPEND serial_ports
				/dev/ttyS*
				)
		elseif(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
			foreach(port RANGE 32 0)
				list(APPEND serial_ports
					"COM${port}")
			endforeach()
		endif()

		string(REPLACE ";" "," serial_ports "${serial_ports}")

		add_custom_target(upload
			COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${fw_package}
			DEPENDS ${fw_package}
			COMMENT "uploading px4"
			VERBATIM
			USES_TERMINAL
			WORKING_DIRECTORY ${PX4_BINARY_DIR}
			)

		add_custom_target(force-upload
			COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_uploader.py --force --port ${serial_ports} ${fw_package}
			DEPENDS ${fw_package}
			COMMENT "uploading px4 with --force"
			VERBATIM
			USES_TERMINAL
			WORKING_DIRECTORY ${PX4_BINARY_DIR}
			)
	endif()

endif()

# print size
add_custom_target(size
	COMMAND size $<TARGET_FILE:px4>
	DEPENDS px4
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	USES_TERMINAL
	)

# print weak symbols
add_custom_target(weak_symbols
	COMMAND ${CMAKE_NM} $<TARGET_FILE:px4> | ${GREP} " w " | cat
	DEPENDS px4
	VERBATIM
	USES_TERMINAL
	)

add_custom_target(stack_check
	COMMAND ${CMAKE_COMMAND} -E make_directory stack_usage && ${CMAKE_OBJDUMP} -d $<TARGET_FILE:px4> | ${PX4_SOURCE_DIR}/Tools/stack_usage/checkstack.pl arm 0 > stack_usage/checkstack_output.txt 2> stack_usage/checkstack_errors.txt
	COMMAND ${CMAKE_COMMAND} -E echo ""
	COMMAND ${CMAKE_COMMAND} -E echo ""
	COMMAND ${CMAKE_COMMAND} -E echo "Top 50:"
	COMMAND ${CMAKE_COMMAND} -E echo "--------------------------------------------------------------------------------"
	COMMAND head -n 50 stack_usage/checkstack_output.txt | c++filt
	COMMAND ${CMAKE_COMMAND} -E echo ""
	COMMAND ${CMAKE_COMMAND} -E echo "Symbols with 'run', 'task', 'thread', 'main', 'update':"
	COMMAND ${CMAKE_COMMAND} -E echo "--------------------------------------------------------------------------------"
    COMMAND cat stack_usage/checkstack_output.txt | c++filt | grep -E 'run|task|thread|main|update'
	DEPENDS px4
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	VERBATIM
	)

find_program(BLOATY_PROGRAM bloaty)
if (BLOATY_PROGRAM)
	# bloaty symbols
	add_custom_target(bloaty_symbols
		COMMAND ${BLOATY_PROGRAM} -d symbols -C full -n 50 -s vm $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty compilation units
	add_custom_target(bloaty_compileunits
		COMMAND ${BLOATY_PROGRAM} -d compileunits -C full -n 50 -s vm $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty templates
	add_custom_target(bloaty_templates
		COMMAND ${BLOATY_PROGRAM} -d shortsymbols,fullsymbols -n 50 $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty inlines
	add_custom_target(bloaty_inlines
		COMMAND ${BLOATY_PROGRAM} -d inlines -C full -n 50 $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty compare with last master build
	add_custom_target(bloaty_compare_master
		#COMMAND wget --no-verbose https://s3.amazonaws.com/px4-travis/Firmware/master/${FW_NAME} -O master_${FW_NAME}
		COMMAND wget --no-verbose https://s3.amazonaws.com/px4-travis/Firmware/master/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}.elf -O master_${FW_NAME}
		COMMAND ${BLOATY_PROGRAM} -d symbols -n 50 -C full -s vm $<TARGET_FILE:px4> -- master_${FW_NAME}
		DEPENDS px4
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		VERBATIM
		USES_TERMINAL
		)
endif()

# debugger helpers

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/gdbinit.in ${PX4_BINARY_DIR}/.gdbinit)

add_custom_target(debug
	COMMAND ${GDB} -iex 'set auto-load safe-path ${PX4_BINARY_DIR}' $<TARGET_FILE:px4>
	DEPENDS px4 ${PX4_BINARY_DIR}/.gdbinit
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	USES_TERMINAL
	)

file(GLOB_RECURSE black_magic_probe_path
     FOLLOW_SYMLINKS
     /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_*-if00
     )
file(GLOB_RECURSE black_magic_probe_console_path
     FOLLOW_SYMLINKS
     /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_*-if02
     )

if(black_magic_probe_path)

	add_custom_target(blackmagic_debug
		COMMAND ${GDB} --nh
			-iex 'set auto-load safe-path ${PX4_BINARY_DIR}'
			-ex 'target extended-remote ${black_magic_probe_path}'
			-ex 'monitor version'
			-ex 'monitor connect_srst enable'
			-ex 'monitor swdp_scan'
			-ex 'attach 1'
			-ex 'load'
			-ex 'run'
			$<TARGET_FILE:px4>
		DEPENDS px4 ${PX4_BINARY_DIR}/.gdbinit
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		)

	add_custom_target(blackmagic_upload
		COMMAND ${GDB} --nx --batch
			-ex 'target extended-remote ${black_magic_probe_path}'
			-ex 'monitor version'
			-ex 'monitor connect_srst enable'
			-ex 'monitor swdp_scan'
			-ex 'attach 1'
			-ex 'load'
			-ex 'kill'
			$<TARGET_FILE:px4>
		DEPENDS px4
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
		COMMENT "Uploading with Black Magic Probe"
		)

	add_custom_target(blackmagic_console
		COMMAND screen -t "${PX4_BOARD} console" ${black_magic_probe_console_path} 57600 8N1
		USES_TERMINAL
		)

endif()

add_custom_target(jlink_upload
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_start.sh
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_upload.sh $<TARGET_FILE:px4>
	DEPENDS px4
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	USES_TERMINAL
	)

add_custom_target(jlink_debug
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Debug/jlink_gdb_start.sh
	COMMAND ${GDB} -nh
		-iex 'set auto-load safe-path ${PX4_BINARY_DIR}'
		-ex 'target remote localhost:2331'
		-ex 'monitor reset 0'
		-ex 'load'
		-ex 'compare-sections'
		-ex 'monitor reset 0'
		-ex 'monitor sleep 1000'
		-ex 'monitor go'
		-ex 'continue'
		$<TARGET_FILE:px4>
	DEPENDS px4 ${PX4_BINARY_DIR}/.gdbinit
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	USES_TERMINAL
	)

# Poor man's profiler
include(ExternalProject)
ExternalProject_Add(FlameGraph
	GIT_REPOSITORY "https://github.com/brendangregg/FlameGraph.git"
	UPDATE_COMMAND ""
	PATCH_COMMAND ""
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
	EXCLUDE_FROM_ALL 1
	)

add_custom_target(profile
	COMMAND ${CMAKE_COMMAND} -E env PATH="${PX4_BINARY_DIR}/external/Source/FlameGraph:$ENV{PATH}"
		${CMAKE_CURRENT_SOURCE_DIR}/Debug/poor-mans-profiler.sh --elf=$<TARGET_FILE:px4> --nsamples=10000
	DEPENDS px4 ${CMAKE_CURRENT_SOURCE_DIR}/Debug/poor-mans-profiler.sh FlameGraph
	USES_TERMINAL
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)
