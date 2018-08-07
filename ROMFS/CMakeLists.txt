############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

message(STATUS "ROMFS: ${config_romfs_root}")

set(romfs_src_dir ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root})
set(romfs_gen_root_dir ${PX4_BINARY_DIR}/genromfs)

set_property(GLOBAL PROPERTY PX4_ROMFS_FILES)
set_property(GLOBAL PROPERTY PX4_ROMFS_CMAKE_FILES)

#=============================================================================
#
#	px4_add_romfs_files
#
#	This function builds a list of files to be included in the ROMFS.
#
#	Usage:
#		px4_add_romfs_files(<list of files in current directory to be added to the project>)
#
#	Output:
#		list config_romfs_files_list populated with a list of ROMFS src files including their full file path.
#
#	Example:
#		px4_add_module(
#			<ROMFS_file_1>
#			<ROMFS_file_2>
#			<ROMFS_file_3>
#			)
#
function(px4_add_romfs_files)
	set_property(GLOBAL APPEND PROPERTY PX4_ROMFS_CMAKE_FILES ${CMAKE_CURRENT_LIST_FILE})

	foreach(arg IN LISTS ARGN)
		set_property(GLOBAL APPEND PROPERTY PX4_ROMFS_FILES ${CMAKE_CURRENT_LIST_DIR}/${arg})
	endforeach()
endfunction()

# get list of all ROMFS files
add_subdirectory(${romfs_src_dir})

# directory setup
# copy all romfs files, process airframes, prune comments
get_property(romfs_cmake_files GLOBAL PROPERTY PX4_ROMFS_CMAKE_FILES)
get_property(romfs_copy_files GLOBAL PROPERTY PX4_ROMFS_FILES)
add_custom_command(OUTPUT ${romfs_gen_root_dir}/init.d/rcS ${romfs_gen_root_dir}/init.d/rc.autostart
	COMMAND ${CMAKE_COMMAND} -E remove_directory ${romfs_gen_root_dir}
	# TODO: we should only copy the files in ${romfs_copy_files}
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${romfs_src_dir} ${romfs_gen_root_dir}
	COMMAND ${CMAKE_COMMAND} -E remove_directory ${romfs_gen_root_dir}/init.d-posix
	COMMAND ${CMAKE_COMMAND} -E remove_directory ${romfs_gen_root_dir}/mixers-sitl
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		--airframes-path ${romfs_gen_root_dir}/init.d
		--start-script ${romfs_gen_root_dir}/init.d/rc.autostart
		--board ${BOARD}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py
		--folder ${romfs_gen_root_dir} --board ${BOARD}
	DEPENDS
		${romfs_cmake_files}
		${romfs_copy_files}
		${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py
	COMMENT "ROMFS: copying, generating airframes, pruning"
	)

# copy extras into ROMFS
set(extras_dependencies)
# copy px4io binary if configured
if (config_io_board)
	list(APPEND extras_dependencies
		copy_px4io_bin
		${fw_io_bin}
		)
endif()

add_custom_command(OUTPUT romfs_extras.stamp
	COMMAND ${CMAKE_COMMAND} -E make_directory ${romfs_gen_root_dir}/extras/
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${PX4_BINARY_DIR}/romfs_extras/ ${romfs_gen_root_dir}/extras/
	COMMAND ${CMAKE_COMMAND} -E touch romfs_extras.stamp
	DEPENDS
		${romfs_gen_root_dir}/init.d/rcS
		${romfs_gen_root_dir}/init.d/rc.autostart
		${extras_dependencies}
	COMMENT "ROMFS: copying extras"
	)

# create romfs.img
find_program(GENROMFS genromfs)
if(NOT GENROMFS)
    message(FATAL_ERROR "genromfs not found")
endif()
add_custom_command(OUTPUT romfs.img romfs.txt
	COMMAND ${CMAKE_COMMAND} -E remove -f romfs.img romfs.txt
	COMMAND ${GENROMFS} -f romfs.img -d ${romfs_gen_root_dir} -V "NSHInitVol" -v > romfs.txt 2>&1
	DEPENDS
		${romfs_gen_root_dir}/init.d/rcS
		${romfs_gen_root_dir}/init.d/rc.autostart
		romfs_extras.stamp
	COMMENT "ROMFS: generating image"
	)

# create nsh_romfsimg.c
find_program(XXD xxd)
if(NOT XXD)
    message(FATAL_ERROR "xxd not found")
endif()
add_custom_command(OUTPUT nsh_romfsimg.c
	COMMAND ${CMAKE_COMMAND} -E remove -f nsh_romfsimg.c
	COMMAND ${XXD} -i romfs.img nsh_romfsimg.c
	COMMAND sed 's/unsigned/const unsigned/g' nsh_romfsimg.c > nsh_romfsimg.c.tmp && ${CMAKE_COMMAND} -E rename nsh_romfsimg.c.tmp nsh_romfsimg.c
	DEPENDS romfs.img
	)

add_library(romfs STATIC nsh_romfsimg.c)
add_dependencies(romfs prebuild_targets)
set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)
