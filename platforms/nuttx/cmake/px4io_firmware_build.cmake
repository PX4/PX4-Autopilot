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

# Builds the px4io firmware that another target embeds in its ROMFS.
#
#   cmake -DPX4_SOURCE_DIR=<dir> -DBOARD_IO=<vendor>_<model>_<label>
#         [-DPYTHON_EXECUTABLE=<python>] -P px4io_firmware_build.cmake
#
# NuttX builds inside its source tree, and the nuttx and apps submodules hold
# the build state of the target that is being built. The IO firmware therefore
# builds against its own copy of those sources:
#
#   build/px4io/<BOARD_IO>/nuttx, apps   copy of the tracked NuttX sources
#   build/px4io/<BOARD_IO>/build         cmake build of the BOARD_IO target
#
# The directory is shared by every target of the checkout that embeds the same
# BOARD_IO, and is created again when the NuttX sources change. It runs on every
# build of the embedding target, so a build with nothing to do has to stay cheap.

foreach(required PX4_SOURCE_DIR BOARD_IO)
	if(NOT ${required})
		message(FATAL_ERROR "${required} must be set")
	endif()
endforeach()

set(nuttx_src_dir ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX)
set(io_dir ${PX4_SOURCE_DIR}/build/px4io/${BOARD_IO})
set(io_build_dir ${io_dir}/build)
set(sources_stamp ${io_dir}/nuttx_sources.stamp)

# Run a command quietly, and show its output only when it fails.
function(px4io_run what)
	execute_process(
		COMMAND ${ARGN}
		RESULT_VARIABLE result
		OUTPUT_VARIABLE output
		ERROR_VARIABLE output
	)
	if(NOT result EQUAL 0)
		message(FATAL_ERROR "px4io ${BOARD_IO}: ${what} failed (${result})\n${output}")
	endif()
endfunction()

function(px4io_git result tree)
	execute_process(
		COMMAND git -C ${nuttx_src_dir}/${tree} ${ARGN}
		RESULT_VARIABLE git_result
		OUTPUT_VARIABLE git_output
		ERROR_VARIABLE git_error
	)
	if(NOT git_result EQUAL 0)
		message(FATAL_ERROR "px4io ${BOARD_IO}: git ${ARGN} failed in ${tree}\n${git_error}")
	endif()
	set(${result} "${git_output}" PARENT_SCOPE)
endfunction()

# The NuttX sources are identified by the submodule commits and by any
# uncommitted change to a tracked file, so that NuttX work in progress reaches
# the IO firmware too.
set(sources_id "")
foreach(tree nuttx apps)
	px4io_git(head ${tree} rev-parse HEAD)
	px4io_git(changes ${tree} diff --binary HEAD)
	string(SHA256 changes "${changes}")
	string(STRIP "${head}" head)
	string(APPEND sources_id "${tree} ${head} ${changes}\n")
endforeach()

set(previous_sources_id "")
if(EXISTS ${sources_stamp})
	file(READ ${sources_stamp} previous_sources_id)
endif()

if(NOT "${previous_sources_id}" STREQUAL "${sources_id}")
	message(STATUS "px4io ${BOARD_IO}: copying NuttX sources to ${io_dir}")
	file(REMOVE_RECURSE ${io_dir})

	foreach(tree nuttx apps)
		file(MAKE_DIRECTORY ${io_dir}/${tree})
		px4io_run("copying ${tree}"
			git -C ${nuttx_src_dir}/${tree} checkout-index --all --force --prefix=${io_dir}/${tree}/)

		# checkout-index copies what is staged, bring the working tree changes along
		px4io_git(changed_files ${tree} diff --name-only HEAD)
		string(REPLACE "\n" ";" changed_files "${changed_files}")
		foreach(changed_file ${changed_files})
			if(EXISTS ${nuttx_src_dir}/${tree}/${changed_file})
				px4io_run("copying ${tree}/${changed_file}"
					${CMAKE_COMMAND} -E copy ${nuttx_src_dir}/${tree}/${changed_file} ${io_dir}/${tree}/${changed_file})
			else()
				file(REMOVE ${io_dir}/${tree}/${changed_file})
			endif()
		endforeach()
	endforeach()

	file(WRITE ${sources_stamp} "${sources_id}")
endif()

# Settings meant for the embedding target must not reach the IO firmware.
set(io_env ${CMAKE_COMMAND} -E env --unset=PX4_EXTRA_NUTTX_CONFIG)

if(NOT EXISTS ${io_build_dir}/build.ninja)
	set(configure_args)
	if(PYTHON_EXECUTABLE)
		list(APPEND configure_args -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE})
	endif()
	px4io_run("configure"
		${io_env} ${CMAKE_COMMAND} -S ${PX4_SOURCE_DIR} -B ${io_build_dir} -G Ninja
			-DCONFIG=${BOARD_IO} -DPX4_NUTTX_TREE_DIR=${io_dir} ${configure_args})
endif()

px4io_run("build" ${io_env} ${CMAKE_COMMAND} --build ${io_build_dir} --target ${BOARD_IO}.bin)
