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

add_subdirectory(src)

if (NOT FW_NAME)
	set(FW_NAME ${CONFIG}.elf)
endif()

add_executable(${FW_NAME} ${PX4_SOURCE_DIR}/src/platforms/empty.c)
add_dependencies(${FW_NAME} git_nuttx nuttx_build)

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
	nuttx_cxx
	nuttx_drivers
	nuttx_fs
	nuttx_mm
	nuttx_sched
	)

if (CONFIG_NET)
	list(APPEND nuttx_libs nuttx_net)
endif()

if (NOT LD_SCRIPT)
	set(LD_SCRIPT ld.script)
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
target_link_libraries(nuttx_cxx INTERFACE nuttx_c)

target_link_libraries(${FW_NAME} PRIVATE

	-nodefaultlibs
	-nostdlib

	-fno-exceptions
	-fno-rtti

	-Wl,--script=${PX4_BINARY_DIR_CYG}/NuttX/nuttx/configs/${BOARD}/scripts/${LD_SCRIPT}
	-Wl,-Map=${CONFIG}.map
	-Wl,--warn-common
	-Wl,--gc-sections

	-Wl,--start-group
	${nuttx_libs}
	-Wl,--end-group

	m
	)

target_link_libraries(${FW_NAME} PRIVATE ${module_libraries})
target_link_libraries(${FW_NAME} PRIVATE modules__uORB)

if (config_romfs_root)
	add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
	target_link_libraries(${FW_NAME} PRIVATE romfs)
endif()

# create px4 file (combined firmware and metadata)
# for historical reasons we name the final output binary without nuttx_
set(fw_name_short)
string(REPLACE "nuttx_" "" fw_name_short ${FW_NAME})

set(fw_file ${PX4_BINARY_DIR}/${fw_name_short})
string(REPLACE ".elf" ".px4" fw_file ${fw_file})

add_custom_command(OUTPUT ${PX4_BINARY_DIR_REL}/${BOARD}.bin
	COMMAND ${OBJCOPY} -O binary ${PX4_BINARY_DIR_REL}/${FW_NAME} ${PX4_BINARY_DIR_REL}/${BOARD}.bin
	DEPENDS ${FW_NAME}
	)

if (NOT FW_PROTOTYPE)
	set(FW_PROTOTYPE ${BOARD})
endif()

if (TARGET parameters_xml AND TARGET airframes_xml)
	add_custom_command(OUTPUT ${fw_file}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${PX4_SOURCE_DIR}/platforms/nuttx/Images/${FW_PROTOTYPE}.prototype
			--git_identity ${PX4_SOURCE_DIR}
			--parameter_xml ${PX4_BINARY_DIR}/parameters.xml
			--airframe_xml ${PX4_BINARY_DIR}/airframes.xml
			--image ${PX4_BINARY_DIR}/${BOARD}.bin > ${fw_file}
		DEPENDS ${PX4_BINARY_DIR}/${BOARD}.bin parameters_xml airframes_xml
		COMMENT "Creating ${fw_file}"
		)

	add_custom_target(px4 ALL DEPENDS ${fw_file})

	# upload helper
	if (${BOARD} STREQUAL "aerofc-v1")
		
		add_custom_target(upload
			COMMAND ${PX4_SOURCE_DIR}/Tools/aero_upload.sh ${fw_file}
			DEPENDS ${fw_file}
			COMMENT "uploading px4"
			USES_TERMINAL
		)
	else()
		set(serial_ports)
		if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux")
			list(APPEND serial_ports
				/dev/serial/by-id/*_PX4_*
				/dev/serial/by-id/usb-3D_Robotics*
				/dev/serial/by-id/usb-The_Autopilot*
				/dev/serial/by-id/usb-Bitcraze*
				/dev/serial/by-id/pci-3D_Robotics*
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
		px4_join(OUT serial_ports LIST "${serial_ports}" GLUE ",")
		add_custom_target(upload
			COMMAND ${PYTHON_EXECUTABLE}
				${PX4_SOURCE_DIR}/Tools/px_uploader.py --port ${serial_ports} ${fw_file}
			DEPENDS ${fw_file}
			COMMENT "uploading px4"
			VERBATIM
			USES_TERMINAL
			)
	endif()
endif()

# print size
add_custom_target(size
	COMMAND size ${FW_NAME}
	DEPENDS ${FW_NAME}
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	USES_TERMINAL
	)

# print weak symbols
add_custom_target(weak_symbols
	COMMAND ${CMAKE_NM} $<TARGET_FILE:${FW_NAME}> | ${GREP} " w " | cat
	DEPENDS ${FW_NAME}
	VERBATIM
	USES_TERMINAL
	)

add_custom_target(stack_check
	COMMAND ${CMAKE_COMMAND} -E make_directory stack_usage && ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${FW_NAME}> | ${PX4_SOURCE_DIR}/Tools/stack_usage/checkstack.pl arm 0 > stack_usage/checkstack_output.txt 2> stack_usage/checkstack_errors.txt
	COMMAND ${CMAKE_COMMAND} -E echo ""
	COMMAND ${CMAKE_COMMAND} -E echo ""
	COMMAND ${CMAKE_COMMAND} -E echo "Top 50:"
	COMMAND ${CMAKE_COMMAND} -E echo "--------------------------------------------------------------------------------"
	COMMAND head -n 50 stack_usage/checkstack_output.txt | c++filt
	COMMAND ${CMAKE_COMMAND} -E echo ""
	COMMAND ${CMAKE_COMMAND} -E echo "Symbols with 'run', 'task', 'thread', 'main', 'update':"
	COMMAND ${CMAKE_COMMAND} -E echo "--------------------------------------------------------------------------------"
    COMMAND cat stack_usage/checkstack_output.txt | c++filt | grep -E 'run|task|thread|main|update'
	DEPENDS ${FW_NAME}
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	VERBATIM
	)

find_program(BLOATY_PROGRAM bloaty)
if (BLOATY_PROGRAM)
	# bloaty symbols
	add_custom_target(bloaty_symbols
		COMMAND ${BLOATY_PROGRAM} -d symbols -C full -n 50 -s vm $<TARGET_FILE:${FW_NAME}>
		DEPENDS ${FW_NAME}
		USES_TERMINAL
		)

	# bloaty compilation units
	add_custom_target(bloaty_compileunits
		COMMAND ${BLOATY_PROGRAM} -d compileunits -C full -n 50 -s vm $<TARGET_FILE:${FW_NAME}>
		DEPENDS ${FW_NAME}
		USES_TERMINAL
		)

	# bloaty templates
	add_custom_target(bloaty_templates
		COMMAND ${BLOATY_PROGRAM} -d shortsymbols,fullsymbols -n 50 $<TARGET_FILE:${FW_NAME}>
		DEPENDS ${FW_NAME}
		USES_TERMINAL
		)

	# bloaty inlines
	add_custom_target(bloaty_inlines
		COMMAND ${BLOATY_PROGRAM} -d inlines -C full -n 50 $<TARGET_FILE:${FW_NAME}>
		DEPENDS ${FW_NAME}
		USES_TERMINAL
		)

	# bloaty compare with last master build
	add_custom_target(bloaty_compare_master
		COMMAND wget --no-verbose https://s3.amazonaws.com/px4-travis/Firmware/master/${FW_NAME} -O master_${FW_NAME}
		COMMAND ${BLOATY_PROGRAM} -d symbols -n 50 -C full -s vm $<TARGET_FILE:${FW_NAME}> -- master_${FW_NAME}
		DEPENDS ${FW_NAME}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		VERBATIM
		USES_TERMINAL
		)
endif()

# debugger helpers
configure_file(gdbinit.in .gdbinit)

add_custom_target(debug
	COMMAND ${GDB} $<TARGET_FILE:${FW_NAME}>
	DEPENDS ${FW_NAME} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit
	USES_TERMINAL
	)
	
add_custom_target(profile
	COMMAND ${PX4_SOURCE_DIR}/platforms/nuttx/Debug/poor-mans-profiler.sh --elf=$<TARGET_FILE:${FW_NAME}> --nsamples=10000
	DEPENDS ${FW_NAME} ${PX4_SOURCE_DIR}/platforms/nuttx/Debug/poor-mans-profiler.sh
	USES_TERMINAL
	)
