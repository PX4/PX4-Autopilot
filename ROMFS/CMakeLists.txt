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

set(config_romfs_files_list)
set(romfs_src_dir ${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root})
set(romfs_temp_dir ${PX4_BINARY_DIR}/genromfs/${config_romfs_root})

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
	foreach(arg IN LISTS ARGN)
		list(APPEND config_romfs_files_list
			${CMAKE_CURRENT_SOURCE_DIR}/${arg}
			)
	endforeach()

	set(config_romfs_files_list ${config_romfs_files_list} CACHE INTERNAL "ROMFS file list")
endfunction()

# get list of all ROMFS files
add_subdirectory(${romfs_src_dir})

# directory setup
# copy all romfs files, process airframes, prune comments
add_custom_command(OUTPUT ${romfs_temp_dir}/init.d/rcS ${romfs_temp_dir}/init.d/rc.autostart
	COMMAND cmake -E copy_directory ${romfs_src_dir} ${romfs_temp_dir}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		-a ${romfs_temp_dir}/init.d
		-s ${romfs_temp_dir}/init.d/rc.autostart
		--board ${BOARD}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py
		--folder ${romfs_temp_dir} --board ${BOARD}
	DEPENDS
		${config_romfs_files_list}
		${PX4_SOURCE_DIR}/Tools/px_romfs_pruner.py
		${PX4_SOURCE_DIR}/ROMFS/${config_romfs_root}/init.d/rcS
		${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
	)

set(romfs_dependencies)
list(APPEND romfs_dependencies
	${romfs_temp_dir}/init.d/rcS
	${romfs_temp_dir}/init.d/rc.autostart
	)
	
# copy px4io binary if configured
if (config_io_board)
	list(APPEND romfs_dependencies
		copy_px4io_bin
		${fw_io_bin}
		)
endif()

# create romfs.img
add_custom_command(OUTPUT romfs.img
	COMMAND ${CMAKE_COMMAND} -E remove -f romfs.img romfs.txt
	COMMAND ${GENROMFS} -f romfs.img -d ${romfs_temp_dir} -V "NSHInitVol" -v > romfs.txt 2>&1
	DEPENDS ${romfs_dependencies}
	)

# create nsh_romfsimg.c
add_custom_command(OUTPUT nsh_romfsimg.c
	COMMAND ${CMAKE_COMMAND} -E remove -f nsh_romfsimg.c
	COMMAND xxd -i romfs.img nsh_romfsimg.c
	COMMAND sed 's/unsigned/const unsigned/g' nsh_romfsimg.c > nsh_romfsimg.c.tmp && mv nsh_romfsimg.c.tmp nsh_romfsimg.c
	DEPENDS romfs.img
	)

add_library(romfs STATIC nsh_romfsimg.c)
add_dependencies(romfs prebuild_targets)
set_target_properties(romfs PROPERTIES LINKER_LANGUAGE C)
