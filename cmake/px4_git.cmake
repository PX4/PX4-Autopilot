############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

# cmake include guard
if(px4_git_included)
	return()
endif(px4_git_included)
set(px4_git_included true)

# Check all submodules once per configure, whether the build started from
# make, an IDE or CMake directly.
execute_process(
	COMMAND Tools/check_submodules.sh ${PX4_GIT_SUBMODULES}
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	RESULT_VARIABLE check_submodules_result
	)
if(NOT check_submodules_result EQUAL 0)
	message(FATAL_ERROR "Tools/check_submodules.sh failed")
endif()

#=============================================================================
#
#	px4_add_git_submodule
#
#	This function adds a target for a git submodule, which
#	Tools/check_submodules.sh has already checked out.
#
#	Usage:
#		px4_add_git_submodule(TARGET <target> PATH <path>)
#
#	Input:
#		PATH		: git submodule path
#
#	Output:
#		TARGET		: git target
#
#	Example:
#		px4_add_git_submodule(TARGET git_nuttx PATH "NuttX")
#
function(px4_add_git_submodule)
	px4_parse_function_args(
		NAME px4_add_git_submodule
		ONE_VALUE TARGET PATH
		REQUIRED TARGET PATH
		ARGN ${ARGN})

	# With PX4_GIT_SUBMODULES set, a submodule the build uses but that is not
	# listed is not fetched: stop here rather than on a missing header later.
	if(PX4_GIT_SUBMODULES)
		get_filename_component(abs_path ${PATH} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
		if(NOT EXISTS ${abs_path}/.git)
			file(RELATIVE_PATH rel_path ${PX4_SOURCE_DIR} ${abs_path})
			message(FATAL_ERROR "Submodule ${rel_path} is used by this build but not checked out, add it to PX4_GIT_SUBMODULES")
		endif()
	endif()

	add_custom_target(${TARGET})
endfunction()
