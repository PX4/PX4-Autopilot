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

	add_custom_command(OUTPUT ${OUT}
		COMMAND ${OBJCOPY} -O binary ${EXE} ${EXE}.bin
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tools/px_mkfw.py
			--prototype ${CMAKE_SOURCE_DIR}/Images/${BOARD}.prototype
			--git_identity ${CMAKE_SOURCE_DIR}
			${extra_args}
			--image ${EXE}.bin > ${OUT}
		DEPENDS ${EXE}
		)
	add_custom_target(build_firmware_${BOARD} ALL DEPENDS ${OUT})
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
	configure_file(${CMAKE_SOURCE_DIR}/cmake/nuttx/builtin_commands.c.in
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

	set(nuttx_src ${CMAKE_BINARY_DIR}/${CONFIG}/NuttX)

	# patch
	add_custom_target(__nuttx_patch_${CONFIG})
	file(GLOB nuttx_patches RELATIVE ${CMAKE_SOURCE_DIR}
	    ${CMAKE_SOURCE_DIR}/nuttx-patches/*.patch)
	foreach(patch ${nuttx_patches})
		string(REPLACE "/" "_" patch_name "${patch}-${CONFIG}")
	    message(STATUS "nuttx-patch: ${patch}")
		add_custom_command(OUTPUT nuttx_patch_${patch_name}.stamp
			COMMAND ${PATCH} -p0 -N  < ${CMAKE_SOURCE_DIR}/${patch}
			COMMAND ${TOUCH} nuttx_patch_${patch_name}.stamp
			DEPENDS ${DEPENDS}
			)
	    add_custom_target(nuttx_patch_${patch_name}
			DEPENDS nuttx_patch_${patch_name}.stamp)
	    add_dependencies(nuttx_patch nuttx_patch_${patch_name})
	endforeach()

	# copy
	add_custom_command(OUTPUT nuttx_copy_${CONFIG}.stamp
		COMMAND ${MKDIR} -p ${CMAKE_BINARY_DIR}/${CONFIG}
		COMMAND ${MKDIR} -p ${nuttx_src}
		COMMAND ${CP} -a ${CMAKE_SOURCE_DIR}/NuttX/. ${nuttx_src}/
		COMMAND ${RM} -rf ${nuttx_src}/.git
		COMMAND ${TOUCH} nuttx_copy_${CONFIG}.stamp
		DEPENDS ${DEPENDS})
	add_custom_target(__nuttx_copy_${CONFIG}
		DEPENDS nuttx_copy_${CONFIG}.stamp __nuttx_patch_${CONFIG})

	# export
	file(GLOB_RECURSE config_files ${CMAKE_SOURCE_DIR}/nuttx-configs/${CONFIG}/*)
	add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/${CONFIG}.export
		COMMAND ${ECHO} Configuring NuttX for ${CONFIG}
		COMMAND ${MAKE} --no-print-directory -C${nuttx_src}/nuttx -r --quiet distclean
		COMMAND ${CP} -r ${CMAKE_SOURCE_DIR}/nuttx-configs/${CONFIG} ${nuttx_src}/nuttx/configs
		COMMAND cd ${nuttx_src}/nuttx/tools && ./configure.sh ${CONFIG}/nsh
		COMMAND ${ECHO} Exporting NuttX for ${CONFIG}
		COMMAND ${MAKE} --no-print-directory --quiet -C ${nuttx_src}/nuttx -j${THREADS} -r CONFIG_ARCH_BOARD=${CONFIG} export > /dev/null
		COMMAND ${CP} -r ${nuttx_src}/nuttx/nuttx-export.zip ${CMAKE_BINARY_DIR}/${CONFIG}.export
		DEPENDS ${config_files} ${DEPENDS} __nuttx_copy_${CONFIG})

	# extract
	add_custom_command(OUTPUT nuttx_export_${CONFIG}.stamp
		COMMAND ${RM} -rf ${nuttx_src}/nuttx-export
		COMMAND ${UNZIP} -q ${CMAKE_BINARY_DIR}/${CONFIG}.export -d ${nuttx_src}
		COMMAND ${TOUCH} nuttx_export_${CONFIG}.stamp
		DEPENDS ${DEPENDS} ${CMAKE_BINARY_DIR}/${CONFIG}.export)

	add_custom_target(${OUT}
		DEPENDS nuttx_export_${CONFIG}.stamp)

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

	set(romfs_temp_dir ${CMAKE_BINARY_DIR}/tmp/${ROOT})
	set(romfs_src_dir ${CMAKE_SOURCE_DIR}/${ROOT})
	set(romfs_autostart ${CMAKE_SOURCE_DIR}/Tools/px_process_airframes.py)
	set(romfs_pruner ${CMAKE_SOURCE_DIR}/Tools/px_romfs_pruner.py)
	set(bin_to_obj ${CMAKE_SOURCE_DIR}/cmake/nuttx/bin_to_obj.py)
	set(extras_dir ${CMAKE_CURRENT_BINARY_DIR}/extras)

	file(GLOB_RECURSE romfs_src_files ${romfs_src_dir} ${romfs_src_dir}/*)

	set(cmake_test ${CMAKE_SOURCE_DIR}/cmake/test/cmake_tester.py)

	
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
		COMMAND ${PYTHON_EXECUTABLE} ${romfs_pruner}
			--folder ${romfs_temp_dir}
		COMMAND ${GENROMFS} -f ${CMAKE_CURRENT_BINARY_DIR}/romfs.bin
			-d ${romfs_temp_dir} -V "NSHInitVol"
		#COMMAND cmake -E remove_directory ${romfs_temp_dir}
		COMMAND ${PYTHON_EXECUTABLE} ${bin_to_obj}
			--ld ${LD} --c_flags ${CMAKE_C_FLAGS}
			--include_path "${CMAKE_SOURCE_DIR}/src/include"
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
#	Set ths nuttx build flags.
#
#	Usage:
#		px4_os_add_flags(
#			C_FLAGS <inout-variable>
#			CXX_FLAGS <inout-variable>
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
#		EXE_LINKER_FLAGS		: executable linker flags variable
#		INCLUDE_DIRS			: include directories
#		LINK_DIRS				: link directories
#		DEFINITIONS				: definitions
#
#	Example:
#		px4_os_add_flags(
#			C_FLAGS CMAKE_C_FLAGS
#			CXX_FLAGS CMAKE_CXX_FLAGS
#			EXE_LINKER_FLAG CMAKE_EXE_LINKER_FLAGS
#			INCLUDES <list>)
#
function(px4_os_add_flags)

	set(inout_vars
		C_FLAGS CXX_FLAGS EXE_LINKER_FLAGS INCLUDE_DIRS LINK_DIRS DEFINITIONS)

	px4_parse_function_args(
		NAME px4_add_flags
		ONE_VALUE ${inout_vars} BOARD
		REQUIRED ${inout_vars} BOARD
		ARGN ${ARGN})

	px4_add_common_flags(
		BOARD ${BOARD}
		C_FLAGS ${C_FLAGS}
		CXX_FLAGS ${CXX_FLAGS}
		EXE_LINKER_FLAGS ${EXE_LINKER_FLAGS}
		INCLUDE_DIRS ${INCLUDE_DIRS}
		LINK_DIRS ${LINK_DIRS}
		DEFINITIONS ${DEFINITIONS})

	set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${BOARD}/NuttX/nuttx-export)
	set(added_include_dirs
		${nuttx_export_dir}/include
		${nuttx_export_dir}/include/cxx
		${nuttx_export_dir}/arch/chip
		${nuttx_export_dir}/arch/common
		)
	set(added_link_dirs
		${nuttx_export_dir}/libs
		)
	set(added_definitions
		-D__PX4_NUTTX
		-D__DF_NUTTX
		)
	set(added_c_flags
		-nodefaultlibs
		-nostdlib
		)
	set(added_cxx_flags
		-nodefaultlibs
		-nostdlib
		)

	set(added_exe_linker_flags) # none currently

	set(cpu_flags)
	if (${BOARD} STREQUAL "px4fmu-v1")
		set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${BOARD} STREQUAL "px4fmu-v2")
		set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${BOARD} STREQUAL "px4fmu-v4")
		set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${BOARD} STREQUAL "px4-stm32f4discovery")
		set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${BOARD} STREQUAL "aerocore")
		set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${BOARD} STREQUAL "mindpx-v2")
			set(cpu_flags
			-mcpu=cortex-m4
			-mthumb
			-march=armv7e-m
			-mfpu=fpv4-sp-d16
			-mfloat-abi=hard
			)
	elseif (${BOARD} STREQUAL "px4io-v1")
		set(cpu_flags
			-mcpu=cortex-m3
			-mthumb
			-march=armv7-m
			)
	elseif (${BOARD} STREQUAL "px4io-v2")
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

	set(DF_TARGET "nuttx" PARENT_SCOPE)

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

# vim: set noet fenc=utf-8 ff=unix nowrap:
