############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
#	Defined functions in this file
#
# 	OS Specific Functions
#
#		* px4_nuttx_add_firmware
#		* px4_nuttx_make_uavcan_bootloadable
#		* px4_nuttx_generate_builtin_commands
#		* px4_nuttx_add_export
#		* px4_nuttx_add_romfs
#
# 	Required OS Inteface Functions
#
# 		* px4_os_add_flags
#		* px4_os_prebuild_targets
#

include(common/px4_base)

#=============================================================================
#
#	px4_nuttx_add_firmware
#
#	This function adds a nuttx firmware target.
#
#	Usage:
#		px4_nuttx_add_firmware(OUT <out-target> EXE <in-executable>
#			PARAM_XML <param_xml> AIRFRAMES_XML <airframes_xml>)
#
#	Input:
#		EXE			: the executable to generate the firmware from
#		BOARD		: the board
#		PARAM_XML		: param xml file (optional)
#		AIRFRAMES_XML	: airframes xml file (optional)
#
#	Output:
#		OUT			: the generated firmware target
#
#	Example:
#		px4_nuttx_add_firmware(TARGET fw_test EXE test)
#
function(px4_nuttx_add_firmware)
	px4_parse_function_args(
		NAME px4_nuttx_add_firmware
		ONE_VALUE BOARD OUT EXE PARAM_XML AIRFRAMES_XML
		REQUIRED OUT EXE BOARD
		ARGN ${ARGN})

	set(extra_args)

	if (PARAM_XML)
		list(APPEND extra_args
			--parameter_xml ${PARAM_XML}
			)
	endif()

	if (AIRFRAMES_XML)
		list(APPEND extra_args
			--airframe_xml ${AIRFRAMES_XML}
			)
	endif()

	add_custom_command(OUTPUT ${OUT} ${EXE}.bin
		COMMAND ${OBJCOPY} -O binary ${EXE} ${EXE}.bin
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${PX4_SOURCE_DIR}/Images/${BOARD}.prototype
			--git_identity ${PX4_SOURCE_DIR}
			${extra_args}
			--image ${EXE}.bin > ${OUT}
		DEPENDS ${EXE}
		)
	add_custom_target(build_firmware_${BOARD} ALL DEPENDS ${OUT})
endfunction()

#=============================================================================
#
#	px4_nuttx_make_uavcan_bootloadable
#
#	This function adds a uavcan boot loadable target.
#
#	Usage:
#	  px4_nuttx_make_uavcan_bootloadable(
#	   BOARD	<board>
#	   BIN <input bin file>)
#	   HWNAME <uavcan name>
#	   HW_MAJOR <number>
#	   HW_MINOR <number>
#	   SW_MAJOR <number>
#	   SW_MINOR <number>)
#
#	Input:
#	  BOARD      : the board
#	  BIN        : the bin file to generate the bootloadable image from
#	  HWNAME     : the uavcan name
#	  HW_MAJOR   : the major hardware revision
#	  HW_MINOR   : the minor hardware revision
#	  SW_MAJOR   : the major software revision
#	  SW_MINOR   : the minor software revision
#
#	Output:
#		OUT			: None
#
#	Example:
#	px4_nuttx_make_uavcan_bootloadable(
#	  BOARD ${BOARD}
#		BIN ${CMAKE_CURRENT_BINARY_DIR}/firmware_nuttx
#		HWNAME ${uavcanblid_name}
#		HW_MAJOR ${uavcanblid_hw_version_major}
#		HW_MINOR ${uavcanblid_hw_version_minor}
#		SW_MAJOR ${uavcanblid_sw_version_major}
#		SW_MINOR ${uavcanblid_sw_version_minor}
#	 )
#
function(px4_nuttx_make_uavcan_bootloadable)
	px4_parse_function_args(
		NAME px4_nuttx_make_uavcan_bootloadable
		ONE_VALUE BOARD BIN HWNAME HW_MAJOR HW_MINOR SW_MAJOR SW_MINOR
		REQUIRED BOARD BIN HWNAME HW_MAJOR HW_MINOR SW_MAJOR SW_MINOR
		ARGN ${ARGN})
	string(REPLACE "\"" "" HWNAME ${HWNAME})
	execute_process(
		COMMAND git rev-list HEAD --max-count=1 --abbrev=8 --abbrev-commit
		OUTPUT_VARIABLE uavcanbl_git_desc
		OUTPUT_STRIP_TRAILING_WHITESPACE
		WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)
  if ("${uavcanbl_git_desc}" STREQUAL "")
		set(uavcanbl_git_desc ffffffff)
  endif()
	set(uavcan_bl_imange_name ${HWNAME}-${HW_MAJOR}.${HW_MINOR}-${SW_MAJOR}.${SW_MINOR}.${uavcanbl_git_desc}.uavcan.bin)
	message(STATUS "Generating UAVCAN Bootable as ${uavcan_bl_imange_name}")
	add_custom_command(OUTPUT ${uavcan_bl_imange_name}
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/make_can_boot_descriptor.py
			-v --use-git-hash ${BIN} ${uavcan_bl_imange_name}
		DEPENDS ${BIN})
	add_custom_target(build_uavcan_bl_${BOARD} ALL DEPENDS ${uavcan_bl_imange_name})
endfunction()

#=============================================================================
#
#	px4_nuttx_generate_builtin_commands
#
#	This function generates the builtin_commands.c src for nuttx
#
#	Usage:
#		px4_nuttx_generate_builtin_commands(
#			MODULE_LIST <in-list>
#			OUT <file>)
#
#	Input:
#		MODULE_LIST	: list of modules
#
#	Output:
#		OUT	: generated builtin_commands.c src
#
#	Example:
#		px4_nuttx_generate_builtin_commands(
#			OUT <generated-src> MODULE_LIST px4_simple_app)
#
function(px4_nuttx_generate_builtin_commands)
	px4_parse_function_args(
		NAME px4_nuttx_generate_builtin_commands
		ONE_VALUE OUT
		MULTI_VALUE MODULE_LIST
		REQUIRED MODULE_LIST OUT
		ARGN ${ARGN})
	set(builtin_apps_string)
	set(builtin_apps_decl_string)
	set(command_count 0)
	foreach(module ${MODULE_LIST})
		foreach(property MAIN STACK_MAIN PRIORITY) 
			get_target_property(${property} ${module} ${property})
		endforeach()
		if (MAIN)
			set(builtin_apps_string
				"${builtin_apps_string}\t{\"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main},\n")
			set(builtin_apps_decl_string
				"${builtin_apps_decl_string}extern int ${MAIN}_main(int argc, char *argv[]);\n")
			math(EXPR command_count "${command_count}+1")
		endif()
	endforeach()
	configure_file(${PX4_SOURCE_DIR}/cmake/nuttx/builtin_commands.c.in
		${OUT})
endfunction()

#=============================================================================
#
#	px4_nuttx_add_export
#
#	This function generates a nuttx export.
#
#	Usage:
#		px4_nuttx_add_export(
#			OUT <out-target>
#			CONFIG <in-string>
#			DEPENDS <in-list>)
#
#	Input:
#		CONFIG	: the board to generate the export for
#		DEPENDS	: dependencies
#
#	Output:
#		OUT	: the export target
#
#	Example:
#		px4_nuttx_add_export(OUT nuttx_export CONFIG px4fmu-v2)
#
function(px4_nuttx_add_export)

	px4_parse_function_args(
		NAME px4_nuttx_add_export
		ONE_VALUE OUT CONFIG THREADS
		MULTI_VALUE DEPENDS
		REQUIRED OUT CONFIG THREADS
		ARGN ${ARGN})

	set(nuttx_src ${PX4_BINARY_DIR}/${CONFIG}/NuttX)

	# all patches
	file(GLOB nuttx_patches ${PX4_SOURCE_DIR}/nuttx-patches/*.patch)
	list(SORT nuttx_patches)

	# copy
	file(GLOB_RECURSE nuttx_all_files ${PX4_SOURCE_DIR}/NuttX/*)
	file(RELATIVE_PATH nuttx_cp_src ${PX4_BINARY_DIR} ${PX4_SOURCE_DIR}/NuttX)
	add_custom_command(OUTPUT ${PX4_BINARY_DIR}/nuttx_copy_${CONFIG}.stamp
		COMMAND ${MKDIR} -p ${nuttx_src}
		COMMAND rsync -a --delete --exclude=.git ${nuttx_cp_src}/ ${CONFIG}/NuttX/
		COMMAND ${TOUCH} ${PX4_BINARY_DIR}/nuttx_copy_${CONFIG}.stamp
		DEPENDS ${DEPENDS} ${nuttx_patches} ${nuttx_all_files}
		COMMENT "Copying NuttX for ${CONFIG} with ${config_nuttx_config}"
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		)
	add_custom_target(nuttx_copy_${CONFIG} DEPENDS ${PX4_BINARY_DIR}/nuttx_copy_${CONFIG}.stamp)

	# patch
    unset (last_patch )
	add_custom_target(nuttx_patch_${CONFIG})
	foreach(patch ${nuttx_patches})
		get_filename_component(patch_file_name ${patch} NAME)
		string(REPLACE "/" "_" patch_name "nuttx_patch_${patch_file_name}-${CONFIG}")
		set(patch_stamp ${nuttx_src}/${patch_name}.stamp)

		add_custom_command(OUTPUT ${patch_stamp}
			COMMAND ${PATCH} -d ${nuttx_src} -s -p1 -N < ${patch}
			COMMAND ${TOUCH} ${patch_stamp}
			DEPENDS ${last_patch} ${DEPENDS} nuttx_copy_${CONFIG} ${patch}
			USES_TERMINAL
			COMMENT "${CONFIG} Applying NuttX patch: nuttx-patches/${patch_file_name}")

		add_custom_target(${patch_name} DEPENDS ${patch_stamp})
		add_dependencies(nuttx_patch_${CONFIG} ${patch_name})
        set (last_patch ${patch_name})
	endforeach()

	# Read defconfig to see if CONFIG_ARMV7M_STACKCHECK is yes
	# note: CONFIG will be BOARD in the future evaluation of ${hw_stack_check_${CONFIG}
	file(STRINGS "${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG}/${config_nuttx_config}/defconfig"
		hw_stack_check_${CONFIG}
		REGEX "CONFIG_ARMV7M_STACKCHECK=y"
		)
	if ("${hw_stack_check_${CONFIG}}" STREQUAL "CONFIG_ARMV7M_STACKCHECK=y")
		set(config_nuttx_hw_stack_check_${CONFIG} y CACHE INTERNAL "" FORCE)
	endif()

	# configure
	file(GLOB_RECURSE config_files ${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG}/*)
	add_custom_command(OUTPUT ${nuttx_src}/nuttx/.config
		COMMAND ${CP} -rp ${PX4_SOURCE_DIR}/nuttx-configs/*.mk ${nuttx_src}/nuttx/
		COMMAND ${CP} -rp ${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG} ${nuttx_src}/nuttx/configs
		COMMAND cd ${nuttx_src}/nuttx/tools && sh configure.sh ${CONFIG}/${config_nuttx_config}
		DEPENDS ${DEPENDS} nuttx_patch_${CONFIG} ${config_files}
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "Configuring NuttX for ${CONFIG} with ${config_nuttx_config}")

	# manual reconfigure helpers
	add_custom_target(oldconfig_${CONFIG}
		COMMAND cd ${nuttx_src}/nuttx
		COMMAND ${MAKE} -C ${nuttx_src}/nuttx CONFIG_ARCH_BOARD=${CONFIG} oldconfig
		COMMAND ${CP} ${nuttx_src}/nuttx/.config ${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG}/${config_nuttx_config}/defconfig
		COMMAND ${PX4_SOURCE_DIR}/Tools/nuttx_defconf_tool.sh ${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG}/${config_nuttx_config}/defconfig
		DEPENDS ${nuttx_src}/nuttx/.config
		COMMENT "Running NuttX make oldconfig for ${CONFIG} with ${config_nuttx_config}"
		USES_TERMINAL)

	add_custom_target(menuconfig_${CONFIG}
		COMMAND cd ${nuttx_src}/nuttx
		COMMAND ${MAKE} -C ${nuttx_src}/nuttx CONFIG_ARCH_BOARD=${CONFIG} menuconfig
		COMMAND ${CP} ${nuttx_src}/nuttx/.config ${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG}/${config_nuttx_config}/defconfig
		COMMAND ${PX4_SOURCE_DIR}/Tools/nuttx_defconf_tool.sh ${PX4_SOURCE_DIR}/nuttx-configs/${CONFIG}/${config_nuttx_config}/defconfig
		DEPENDS ${nuttx_src}/nuttx/.config
		COMMENT "Running NuttX make menuconfig for ${CONFIG} with ${config_nuttx_config}"
		USES_TERMINAL)

	# build and export
	add_custom_command(OUTPUT ${nuttx_src}/nuttx/nuttx-export/include/nuttx/config.h
		COMMAND ${RM} -rf ${nuttx_src}/nuttx/nuttx-export
		COMMAND ${MAKE} --no-print-directory --quiet -C ${nuttx_src}/nuttx -r CONFIG_ARCH_BOARD=${CONFIG} export > nuttx_build.log
		DEPENDS ${DEPENDS} ${nuttx_src}/nuttx/.config
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "Building NuttX for ${CONFIG} with ${config_nuttx_config}")

	add_custom_target(${OUT} DEPENDS ${nuttx_src}/nuttx/nuttx-export/include/nuttx/config.h)

endfunction()

#=============================================================================
#
#	px4_nuttx_create_bin
#
#	The functions generates a bin image for nuttx.
#
#	Usage:
#		px4_nuttx_create_bin(BIN <out-file> EXE <in-file>)
#
#	Input:
#		EXE		: the exe file
#
#	Output:
#		OUT		: the binary output file
#
#	Example:
#		px4_nuttx_create_bin(OUT my_exe.bin EXE my_exe)
#
function(px4_nuttx_create_bin)

	px4_parse_function_args(
		NAME px4_nuttx_create_bin
		ONE_VALUE EXE OUT
		REQUIRED EXE OUT
		ARGN ${ARGN})

	add_custom_command(OUTPUT ${OUT}
		COMMAND ${OBJCOPY} -O binary ${EXE} ${EXE}.bin
		DEPENDS ${EXE})

	set(${OUT} ${${OUT}} PARENT_SCOPE)

endfunction()


#=============================================================================
#
#	px4_nuttx_add_romfs
#
#	The functions creates a  ROMFS filesystem for nuttx.
#
#	Usage:
#		px4_nuttx_add_romfs(
#			OUT <out-target>
#			ROOT <in-directory>
#			EXTRAS <in-list>)
#
#	Input:
#		ROOT	: the root of the ROMFS
#		EXTRAS 	: list of extra files
#
#	Output:
#		OUT		: the ROMFS library target
#
#	Example:
#		px4_nuttx_add_romfs(OUT my_romfs ROOT "ROMFS/my_board")
#
function(px4_nuttx_add_romfs)

	px4_parse_function_args(
		NAME px4_nuttx_add_romfs
		ONE_VALUE OUT ROOT
		MULTI_VALUE EXTRAS
		REQUIRED OUT ROOT
		ARGN ${ARGN})

	set(romfs_used y PARENT_SCOPE)
	set(romfs_temp_dir ${PX4_BINARY_DIR}/tmp/${ROOT})
	set(romfs_src_dir ${PX4_SOURCE_DIR}/${ROOT})
	set(romfs_autostart ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py)
	set(romfs_pruner ${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py)
	set(bin_to_obj ${PX4_SOURCE_DIR}/cmake/nuttx/bin_to_obj.py)
	set(extras_dir ${CMAKE_CURRENT_BINARY_DIR}/extras)

	file(GLOB_RECURSE romfs_src_files ${romfs_src_dir} ${romfs_src_dir}/*)

	set(cmake_test ${PX4_SOURCE_DIR}/cmake/test/cmake_tester.py)

	set(extras)
	foreach(extra ${EXTRAS})
		get_filename_component(file_name ${extra} NAME)
		set(file_dest ${extras_dir}/${file_name})
		add_custom_command(OUTPUT ${file_dest}
			COMMAND cmake -E copy ${extra} ${file_dest}
			DEPENDS ${extra}
			)
		list(APPEND extras ${file_dest})
	endforeach()
	add_custom_target(collect_extras DEPENDS ${extras})

	add_custom_command(OUTPUT romfs.o
		COMMAND cmake -E remove_directory ${romfs_temp_dir}
		COMMAND cmake -E copy_directory ${romfs_src_dir} ${romfs_temp_dir}
		COMMAND cmake -E copy_directory ${extras_dir} ${romfs_temp_dir}/extras
		COMMAND ${PYTHON_EXECUTABLE} ${romfs_autostart}
			-a ${romfs_temp_dir}/init.d
			-s ${romfs_temp_dir}/init.d/rc.autostart
			--board ${BOARD}
		COMMAND ${PYTHON_EXECUTABLE} ${romfs_pruner}
			--folder ${romfs_temp_dir}
			--board ${BOARD}
		COMMAND ${GENROMFS} -f ${CMAKE_CURRENT_BINARY_DIR}/romfs.bin
			-d ${romfs_temp_dir} -V "NSHInitVol"
		#COMMAND cmake -E remove_directory ${romfs_temp_dir}
		COMMAND ${PYTHON_EXECUTABLE} ${bin_to_obj}
			--ld ${LD} --c_flags ${CMAKE_C_FLAGS}
			--include_path "${PX4_SOURCE_DIR}/src/include"
			--c_compiler ${CMAKE_C_COMPILER}
			--nm ${NM} --objcopy ${OBJCOPY}
			--obj romfs.o
			--var romfs_img
			--bin romfs.bin
		DEPENDS ${romfs_src_files} ${extras}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		)
	add_library(${OUT} STATIC romfs.o)
	set_target_properties(${OUT} PROPERTIES LINKER_LANGUAGE C)
	set(${OUT} ${${OUT}} PARENT_SCOPE)

endfunction()

#=============================================================================
#
#	px4_os_add_flags
#
#	Set the nuttx build flags.
#
#	Usage:
#		px4_os_add_flags(
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
#			OPTIMIZATION_FLAGS <inout-variable>
#			EXE_LINKER_FLAGS <inout-variable>
#			INCLUDE_DIRS <inout-variable>
#			LINK_DIRS <inout-variable>
#			DEFINITIONS <inout-variable>)
#
#	Input:
#		BOARD					: flags depend on board/nuttx config
#
#	Input/Output: (appends to existing variable)
#		C_FLAGS					: c compile flags variable
#		CXX_FLAGS				: c++ compile flags variable
#		OPTIMIZATION_FLAGS			: optimization compile flags variable
#		EXE_LINKER_FLAGS			: executable linker flags variable
#		INCLUDE_DIRS				: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Note that EXE_LINKER_FLAGS is not suitable for adding libraries because
#	these flags are added before any of the object files and static libraries.
#	Add libraries in src/firmware/nuttx/CMakeLists.txt.
#
#	Example:
#		px4_os_add_flags(
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			OPTIMIZATION_FLAGS optimization_flags
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_os_add_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS OPTIMIZATION_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_os_add_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

	px4_add_common_flags(
		BOARD ${BOARD}
		C_FLAGS ${C_FLAGS}
		CXX_FLAGS ${CXX_FLAGS}
		OPTIMIZATION_FLAGS ${OPTIMIZATION_FLAGS}
		EXE_LINKER_FLAGS ${EXE_LINKER_FLAGS}
		INCLUDE_DIRS ${INCLUDE_DIRS}
		LINK_DIRS ${LINK_DIRS}
		DEFINITIONS ${DEFINITIONS})

	set(nuttx_export_root ${PX4_BINARY_DIR}/${BOARD}/NuttX)
	set(nuttx_export_dir ${nuttx_export_root}/nuttx/nuttx-export)
	set(added_include_dirs
		${nuttx_export_dir}/include
		${nuttx_export_dir}/include/cxx
		${nuttx_export_dir}/arch/chip
		${nuttx_export_dir}/arch/common
		${nuttx_export_dir}/arch/armv7-m
		${nuttx_export_root}/apps/include
		)
	set(added_link_dirs
		${nuttx_export_dir}/libs
		)
	set(added_definitions
		-D__PX4_NUTTX
		)

	if(NOT "${config_nuttx_config}" STREQUAL "bootloader")
		list(APPEND added_definitions -D__DF_NUTTX)
	endif()

	set(added_c_flags
		-nodefaultlibs
		-nostdlib
		)
	set(added_cxx_flags
		-nodefaultlibs
		-nostdlib
		)

	set(added_exe_linker_flags) # none currently

	set(instrument_flags)
	if ("${config_nuttx_hw_stack_check_${BOARD}}" STREQUAL "y")
		set(instrument_flags
			-finstrument-functions
			-ffixed-r10
			)
		list(APPEND c_flags ${instrument_flags})
		list(APPEND cxx_flags ${instrument_flags})
	endif()

	set(cpu_flags)
	if (${config_nuttx_hw} STREQUAL "m7")
		set(cpu_flags
			-mcpu=cortex-m7
			-mthumb
			-mfpu=fpv5-sp-d16
			-mfloat-abi=hard
			)
	elseif (${config_nuttx_hw} STREQUAL "m4")
		set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${config_nuttx_hw} STREQUAL "m3")
		set(cpu_flags
			-mcpu=cortex-m3
			-mthumb
			-march=armv7-m
			)
	endif()
	list(APPEND c_flags ${cpu_flags})
	list(APPEND cxx_flags ${cpu_flags})

	# output
	foreach(var ${inout_vars})
		string(TOLOWER ${var} lower_var)
		set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)
		#message(STATUS "nuttx: set(${${var}} ${${${var}}} ${added_${lower_var}} PARENT_SCOPE)")
	endforeach()

endfunction()

#=============================================================================
#
#	px4_os_prebuild_targets
#
#	This function generates os dependent targets

#	Usage:
#		px4_os_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD 		: board
#		THREADS 	: number of threads for building
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		px4_os_prebuild_targets(OUT target_list BOARD px4fmu-v2)
#
function(px4_os_prebuild_targets)
	px4_parse_function_args(
			NAME px4_os_prebuild_targets
			ONE_VALUE OUT BOARD THREADS
			REQUIRED OUT BOARD
			ARGN ${ARGN})
	px4_nuttx_add_export(OUT nuttx_export_${BOARD}
		CONFIG ${BOARD}
		THREADS ${THREADS}
		DEPENDS git_nuttx)
	add_custom_target(${OUT} DEPENDS nuttx_export_${BOARD})
endfunction()

#=============================================================================
#
#	px4_nuttx_configure
#
#	This function sets the nuttx configuration
#
#	Usage:
#		px4_nuttx_configure(
#	    HWCLASS <m3|m4>
#		  CONFIG <nsh|bootloader
#		  [ROMFS <y|n>
#		  ROMFSROOT <root>]
#			)
#
#	Input:
#	  HWCLASS 		: the class of hardware
#	  CONFIG 		  : the nuttx condufiguration to use
#	  ROMFS 	    : whether or not to use incllude theROMFS
#	  ROMFSROOT	  : If ROMFS used set the root the default is px4fmu_common
#
#	Output:
#		OUT	: None
#
#	Example:
#		px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y)
#
function(px4_nuttx_configure)
	px4_parse_function_args(
			NAME px4_nuttx_configure
			ONE_VALUE HWCLASS CONFIG ROMFS ROMFSROOT
			REQUIRED HWCLASS CONFIG
			ARGN ${ARGN})
	set(config_nuttx_config ${CONFIG} PARENT_SCOPE)
	set(config_nuttx_hw ${HWCLASS} PARENT_SCOPE)
	if ("${ROMFS}" STREQUAL "y")
		set(romfs_used ${ROMFS} PARENT_SCOPE)
		if (NOT DEFINED ROMFSROOT)
			set(config_romfs_root px4fmu_common)
		else()
			set(config_romfs_root ${ROMFSROOT})
		endif()
		set(HASROMFS "with ROMFS on ${config_romfs_root}")
		set(config_romfs_root ${config_romfs_root} PARENT_SCOPE)
	endif()
	message(STATUS "Nuttx build for ${BOARD} on ${HWCLASS} hardware, using ${CONFIG} ${HASROMFS}")
endfunction()

# vim: set noet fenc=utf-8 ff=unix nowrap:
