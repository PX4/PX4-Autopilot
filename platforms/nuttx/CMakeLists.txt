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

if(POLICY CMP0079)
	cmake_policy(SET CMP0079 NEW)
endif()

include(cygwin_cygpath)

add_executable(px4 ${PX4_SOURCE_DIR}/platforms/common/empty.c)
set(FW_NAME ${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}.elf)
set_target_properties(px4 PROPERTIES OUTPUT_NAME ${FW_NAME})
add_dependencies(px4 git_nuttx)

get_property(module_libraries GLOBAL PROPERTY PX4_MODULE_LIBRARIES)

if (NOT CONFIG_BUILD_FLAT)
	add_executable(px4_kernel ${PX4_SOURCE_DIR}/platforms/common/empty.c)
	set(KERNEL_NAME ${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}_kernel.elf)
	set_target_properties(px4_kernel PROPERTIES OUTPUT_NAME ${KERNEL_NAME})
	add_dependencies(px4_kernel git_nuttx)
	get_property(kernel_module_libraries GLOBAL PROPERTY PX4_KERNEL_MODULE_LIBRARIES)
endif()

# build NuttX
add_subdirectory(NuttX ${PX4_BINARY_DIR}/NuttX)

# check that CONFIG_ARCH_BOARD_CUSTOM_DIR is in PX4_BOARD_DIR
if(CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH)
	get_filename_component(nuttx_defconfig_root "${NUTTX_DEFCONFIG}/../.." ABSOLUTE)
	get_filename_component(nuttx_config_from_defconfig "${NUTTX_DIR}/${CONFIG_ARCH_BOARD_CUSTOM_DIR}" ABSOLUTE)

	if(NOT ${nuttx_defconfig_root} MATCHES ${nuttx_config_from_defconfig})
		message(FATAL_ERROR "NuttX custom board directory (${CONFIG_ARCH_BOARD_CUSTOM_DIR}) isn't in board directory (${PX4_BOARD_DIR})")
	endif()
endif()

set(nuttx_libs)
set(SCRIPT_PREFIX)
if("${PX4_BOARD_LABEL}" STREQUAL "bootloader")
	set(SCRIPT_PREFIX ${PX4_BOARD_LABEL}_)
	add_subdirectory(src/bootloader)
	list(APPEND nuttx_libs
		bootloader
		bootloader_lib
		drivers_board
	)
elseif("${PX4_BOARD_LABEL}" STREQUAL "canbootloader")
	set(SCRIPT_PREFIX ${PX4_BOARD_LABEL}_)
	add_subdirectory(src/canbootloader)
	list(APPEND nuttx_libs
		canbootloader
		drivers_board
	)

	target_link_libraries(px4 PRIVATE
		-Wl,-wrap,nxsched_process_timer
		-Wl,-wrap,nxsem_post
		-Wl,-wrap,nxsem_wait
		-Wl,-wrap,nxsem_wait_uninterruptible
		-Wl,-wrap,arm_svcall
		-Wl,-wrap,nx_start
		-Wl,-wrap,exit
	)
else()
	if(NOT "${PX4_BOARD_LINKER_PREFIX}" STREQUAL "")
		set(SCRIPT_PREFIX ${PX4_BOARD_LINKER_PREFIX}-)
	endif()
endif()

list(APPEND nuttx_libs
	nuttx_boards
	nuttx_drivers
	nuttx_fs
	nuttx_sched
	nuttx_crypto
	nuttx_binfmt
	nuttx_xx
	)

if (NOT CONFIG_BUILD_FLAT)
	list(APPEND nuttx_libs
		px4_board_ctrl
		nuttx_karch
		nuttx_kmm
		nuttx_stubs
		nuttx_kc
		)
else()
	list(APPEND nuttx_libs
		nuttx_apps
		nuttx_arch
		nuttx_mm
		nuttx_c
		)
endif()

if(CONFIG_NET)
	list(APPEND nuttx_libs nuttx_net)
	target_link_libraries(nuttx_fs INTERFACE nuttx_net)
endif()

file(RELATIVE_PATH PX4_BINARY_DIR_REL ${CMAKE_CURRENT_BINARY_DIR} ${PX4_BINARY_DIR})

# only in the cygwin environment: convert absolute linker script path to mixed windows (C:/...)
# because even relative linker script paths are different for linux, mac and windows
CYGPATH(NUTTX_CONFIG_DIR NUTTX_CONFIG_DIR_CYG)

if((DEFINED ENV{SIGNING_TOOL}) AND (NOT NUTTX_DIR MATCHES "external"))
	set(PX4_BINARY_OUTPUT ${PX4_BINARY_DIR}/${PX4_CONFIG}_unsigned.bin)

	add_custom_command(OUTPUT ${PX4_BINARY_DIR_REL}/${PX4_CONFIG}.bin
		COMMAND $ENV{SIGNING_TOOL} $ENV{SIGNING_ARGS} ${PX4_BINARY_OUTPUT} ${PX4_BINARY_DIR}/${PX4_CONFIG}.bin
		DEPENDS ${PX4_BINARY_OUTPUT}
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)
else()
	set(PX4_BINARY_OUTPUT ${PX4_BINARY_DIR_REL}/${PX4_CONFIG}.bin)
endif()

if (NOT CONFIG_BUILD_FLAT)

	target_link_libraries(nuttx_karch
		INTERFACE
			drivers_board
			arch_hrt
			)

	target_link_libraries(px4_kernel PRIVATE

		-nostartfiles
		-nodefaultlibs
		-nostdlib
		-nostdinc++

		-fno-exceptions
		-fno-rtti

		-Wl,--script=${NUTTX_CONFIG_DIR}/scripts/${SCRIPT_PREFIX}memory.ld,--script=${NUTTX_CONFIG_DIR}/scripts/${SCRIPT_PREFIX}kernel-space.ld

		-Wl,-Map=${PX4_CONFIG}_kernel.map
		-Wl,--warn-common
		-Wl,--gc-sections

		-Wl,--start-group
			${nuttx_libs}
			${kernel_module_libraries}
		-Wl,--end-group

		m
		gcc
		)

	if (config_romfs_root)
		add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
		target_link_libraries(px4_kernel PRIVATE romfs)
	endif()

	target_link_libraries(px4_kernel PRIVATE -Wl,--print-memory-usage)

	set(nuttx_userspace)

	list(APPEND nuttx_userspace
		drivers_userspace
		nuttx_arch
		nuttx_apps
		nuttx_mm
		nuttx_proxies
		nuttx_c
		nuttx_xx
		)

	target_link_libraries(nuttx_c INTERFACE nuttx_proxies)

	target_link_libraries(px4 PRIVATE

		-nostartfiles
		-nodefaultlibs
		-nostdlib
		-nostdinc++

		-fno-exceptions
		-fno-rtti

		-Wl,--script=${NUTTX_CONFIG_DIR}/scripts/${SCRIPT_PREFIX}memory.ld,--script=${NUTTX_CONFIG_DIR}/scripts/${SCRIPT_PREFIX}user-space.ld

		-Wl,-Map=${PX4_CONFIG}.map
		-Wl,--warn-common
		-Wl,--gc-sections

		-Wl,--start-group
		${nuttx_userspace}
		-Wl,--end-group

		m
		gcc
		)

	target_link_libraries(px4 PRIVATE -Wl,--print-memory-usage)

	target_link_libraries(px4 PRIVATE
		${module_libraries}
	)

	add_custom_command(OUTPUT ${PX4_BINARY_OUTPUT}
		COMMAND ${CMAKE_OBJCOPY} -O binary ${PX4_BINARY_DIR_REL}/${FW_NAME} ${PX4_BINARY_DIR_REL}/${PX4_BOARD}_user.bin
		COMMAND ${CMAKE_OBJCOPY} --gap-fill 0xFF --pad-to ${CONFIG_NUTTX_USERSPACE} -O binary ${PX4_BINARY_DIR_REL}/${KERNEL_NAME} ${PX4_BINARY_OUTPUT}
		COMMAND cat ${PX4_BINARY_DIR_REL}/${PX4_BOARD}_user.bin >> ${PX4_BINARY_OUTPUT}

		DEPENDS px4 px4_kernel
	)

else()

	target_link_libraries(nuttx_c INTERFACE nuttx_sched) # nxsched_get_streams

	target_link_libraries(nuttx_arch
		INTERFACE
			drivers_board
			arch_hrt
			arch_board_reset
	)

	target_link_libraries(nuttx_c INTERFACE nuttx_drivers)
	target_link_libraries(nuttx_drivers INTERFACE nuttx_c)
	target_link_libraries(nuttx_xx INTERFACE nuttx_c)
	target_link_libraries(nuttx_fs INTERFACE nuttx_c)

	target_link_libraries(px4 PRIVATE

		-nostartfiles
		-nodefaultlibs
		-nostdlib
		-nostdinc++

		-fno-exceptions
		-fno-rtti
		-Wl,--script=${NUTTX_CONFIG_DIR_CYG}/scripts/${SCRIPT_PREFIX}script.ld
		-L${NUTTX_CONFIG_DIR_CYG}/scripts
		-Wl,-Map=${PX4_CONFIG}.map
		-Wl,--warn-common
		-Wl,--gc-sections

		-Wl,--start-group
			${nuttx_libs}
		-Wl,--end-group

		m
		gcc
		)

	if(NOT USE_LD_GOLD)
		target_link_libraries(px4 PRIVATE -Wl,--print-memory-usage)
	endif()

	target_link_libraries(px4 PRIVATE ${module_libraries})

	if(config_romfs_root)
		add_subdirectory(${PX4_SOURCE_DIR}/ROMFS ${PX4_BINARY_DIR}/ROMFS)
		target_link_libraries(px4 PRIVATE romfs)
	endif()

	add_custom_command(OUTPUT ${PX4_BINARY_OUTPUT}
		COMMAND ${CMAKE_OBJCOPY} -O binary ${PX4_BINARY_DIR_REL}/${FW_NAME} ${PX4_BINARY_OUTPUT}
		DEPENDS px4
	)

endif()

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
				--image ${PX4_BINARY_DIR}/${PX4_CONFIG}.bin > ${fw_package}
		DEPENDS
			${PX4_BINARY_DIR}/${PX4_CONFIG}.bin
			airframes_xml
			parameters_xml
		COMMENT "Creating ${fw_package}"
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		)

	add_custom_target(px4_package ALL DEPENDS ${fw_package})

	# upload helper
	# create upload target helper if NuttX USB CDCACM is present
	if (CONFIG_CDCACM)
		include(upload)
	endif()

endif()

if(${CONFIG_NSH_LIBRARY} MATCHES "y")
	if(${CONFIG_NSH_DISABLE_ECHO} MATCHES "y")
		message(FATAL_ERROR "init scripts require NSH SET (board has CONFIG_NSH_DISABLE_ECHO=y)")
	endif()
	if(${CONFIG_NSH_DISABLE_SET} MATCHES "y")
		message(FATAL_ERROR "init scripts require NSH SET (board has CONFIG_NSH_DISABLE_SET=y)")
	endif()
	if(${CONFIG_NSH_DISABLE_SLEEP} MATCHES "y")
		message(FATAL_ERROR "init scripts require NSH SET (board has CONFIG_NSH_DISABLE_SLEEP=y)")
	endif()
	if(${CONFIG_NSH_DISABLE_SOURCE} MATCHES "y")
		message(FATAL_ERROR "init scripts require NSH SET (board has CONFIG_NSH_DISABLE_SOURCE=y)")
	endif()
endif()

if("${PX4_BOARD_LABEL}" STREQUAL "bootloader")
	add_custom_command(OUTPUT ${PX4_BOARD_DIR}/extras/${PX4_BOARD}_bootloader.bin
		COMMAND ${CMAKE_OBJCOPY} -O binary ${PX4_BINARY_DIR_REL}/${FW_NAME} ${PX4_BOARD_DIR}/extras/${PX4_BOARD}_bootloader.bin
		DEPENDS px4
	)
	add_custom_target(px4_bootloader_keep ALL DEPENDS ${PX4_BOARD_DIR}/extras/${PX4_BOARD}_bootloader.bin)
endif()

# print weak symbols
add_custom_target(weak_symbols
	COMMAND ${CMAKE_NM} $<TARGET_FILE:px4> | ${GREP} " w " | cat
	DEPENDS px4
	VERBATIM
	USES_TERMINAL
)

# generate bootloader.elf and copy to top level build directory
if(NOT ("${PX4_BOARD_LABEL}" STREQUAL "bootloader") AND (EXISTS "${PX4_BOARD_DIR}/extras/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.bin"))
	add_custom_command(
		OUTPUT ${PX4_BINARY_DIR}/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.elf
		COMMAND
			${CMAKE_OBJCOPY} -I binary -O elf32-little --change-section-address .data=0x08000000
				${PX4_BOARD_DIR}/extras/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.bin
				${PX4_BINARY_DIR}/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.elf
		DEPENDS ${PX4_BOARD_DIR}/extras/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.bin
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)
	add_custom_target(bootloader_elf DEPENDS ${PX4_BINARY_DIR}/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_bootloader.elf)
endif()


# debugger helpers
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/gdbinit.in ${PX4_BINARY_DIR}/.gdbinit)

# vscode launch.json
#  skip if built within another cmake project (eg px4_io-v2 with px4_fmu-v5)
if(NOT NUTTX_DIR MATCHES "external")
	if(CONFIG_ARCH_CHIP_MIMXRT1062DVL6A)
		set(DEBUG_DEVICE "MIMXRT1062XXX6A")
		set(DEBUG_SVD_FILE "MIMXRT1052.svd")
	elseif(CONFIG_ARCH_CHIP_MIMXRT1176DVMAA)
		set(DEBUG_DEVICE "MIMXRT1176DVMAA")
		set(DEBUG_SVD_FILE "MIMXRT1176_cm7.xml")
	elseif(CONFIG_ARCH_CHIP_MK66FN2M0VMD18)
		set(DEBUG_DEVICE "MK66FN2M0xxx18")
		set(DEBUG_SVD_FILE "MK66F18.svd")
	elseif(CONFIG_ARCH_CHIP_S32K344)
		set(DEBUG_DEVICE "S32K344")
		set(DEBUG_SVD_FILE "S32K344_M7.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F100C8)
		set(DEBUG_DEVICE "STM32F100C8")
		set(DEBUG_SVD_FILE "STM32F100xx.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F103RB)
		set(DEBUG_DEVICE "STM32F103C4")
		set(DEBUG_SVD_FILE "STM32F103xx.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F302K8)
		set(DEBUG_DEVICE "STM32F302K8")
		set(DEBUG_SVD_FILE "STM32F302.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F303CC)
		set(DEBUG_DEVICE "STM32F303CC")
		set(DEBUG_SVD_FILE "STM32F303.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F405RG)
		set(DEBUG_DEVICE "STM32F405RG")
		set(DEBUG_SVD_FILE "STM32F405.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F412CE)
		set(DEBUG_DEVICE "STM32F412CE")
		set(DEBUG_SVD_FILE "STM32F412.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F427V)
		set(DEBUG_DEVICE "STM32F427VI")
		set(DEBUG_SVD_FILE "STM32F427.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F429V)
		set(DEBUG_DEVICE "STM32F429VI")
		set(DEBUG_SVD_FILE "STM32F429.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F446R)
		set(DEBUG_DEVICE "STM32F446RC")
		set(DEBUG_SVD_FILE "STM32F446.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F469I)
		set(DEBUG_DEVICE "STM32F469II")
		set(DEBUG_SVD_FILE "STM32F469.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F722RE)
		set(DEBUG_DEVICE "STM32F722RE")
		set(DEBUG_SVD_FILE "STM32F7x2.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F745VG)
		set(DEBUG_DEVICE "STM32F745VG")
		set(DEBUG_SVD_FILE "STM32F7x5.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F765II)
		set(DEBUG_DEVICE "STM32F765II")
		set(DEBUG_SVD_FILE "STM32F7x5.svd")
	elseif(CONFIG_ARCH_CHIP_STM32F777NI)
		set(DEBUG_DEVICE "STM32F777NI")
		set(DEBUG_SVD_FILE "STM32F7x7.svd")
	elseif(CONFIG_ARCH_CHIP_STM32H743VI)
		set(DEBUG_DEVICE "STM32H743VI")
		set(DEBUG_SVD_FILE "STM32H7x3.svd")
	elseif(CONFIG_ARCH_CHIP_STM32H743II)
		set(DEBUG_DEVICE "STM32H743II")
		set(DEBUG_SVD_FILE "STM32H7x3.svd")
	elseif(CONFIG_ARCH_CHIP_STM32H743XI)
		set(DEBUG_DEVICE "STM32H743XI")
		set(DEBUG_SVD_FILE "STM32H7x3.svd")
	elseif(CONFIG_ARCH_CHIP_STM32H743ZI)
		set(DEBUG_DEVICE "STM32H743ZI")
		set(DEBUG_SVD_FILE "STM32H7x3.svd")
	elseif(CONFIG_ARCH_CHIP_STM32H753II)
		set(DEBUG_DEVICE "STM32H753II")
		set(DEBUG_SVD_FILE "STM32H7x3.svd")
	else()
		set(DEBUG_DEVICE "")
		set(DEBUG_SVD_FILE "unknown")
	endif()

	file(GLOB_RECURSE DEBUG_SVD_FILE_PATH
		LIST_DIRECTORIES false
		${CMAKE_SOURCE_DIR}/../cmsis-svd/data/*/${DEBUG_SVD_FILE}
		${CMAKE_SOURCE_DIR}/../svd/${DEBUG_SVD_FILE}
	)

	if(CONFIG_DEBUG_TCBINFO)
		# jlink-nuttx.so
		add_custom_command(
			OUTPUT ${NUTTX_DIR}/tools/jlink-nuttx.so
			COMMAND make --no-print-directory --silent -f Makefile.host jlink-nuttx
			DEPENDS ${NUTTX_DIR}/tools/jlink-nuttx.c
			WORKING_DIRECTORY ${NUTTX_DIR}/tools
		)
		add_custom_target(jlink-nuttx ALL
			DEPENDS ${NUTTX_DIR}/tools/jlink-nuttx.so
		)

		# JLINK_RTOS_PATH used by launch.json.in
		set(JLINK_RTOS_PATH ${NUTTX_DIR}/tools/jlink-nuttx.so)
	else()
		set(JLINK_RTOS_PATH "")
	endif()

	if(DEBUG_SVD_FILE_PATH)
		message(STATUS "Found SVD: ${DEBUG_SVD_FILE_PATH}")
	endif()

	if(DEBUG_DEVICE)
		configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Debug/launch.json.in ${PX4_SOURCE_DIR}/.vscode/launch.json @ONLY)
	endif()
endif()

add_custom_target(debug
	COMMAND ${CMAKE_GDB} -iex 'set auto-load safe-path ${PX4_BINARY_DIR}' $<TARGET_FILE:px4>
	DEPENDS px4 ${PX4_BINARY_DIR}/.gdbinit
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	USES_TERMINAL
)

include(blackmagic)
include(jlink)
include(profile)
include(stack_check)

if(EXISTS "${PX4_BOARD_DIR}/cmake/upload.cmake")
	include(${PX4_BOARD_DIR}/cmake/upload.cmake)
endif()
