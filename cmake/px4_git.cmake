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

#=============================================================================
#
#	px4_add_git_submodule
#
#	This function add a git submodule target.
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

	option(GIT_SUBMODULE "Check submodules during build" ON)

	# ENV GIT_SUBMODULES_ARE_EVIL

	# ENV TERM_PROGRAM="vscode"
	# ENV VSCODE_GIT_ASKPASS_MAIN
	# ENV VSCODE_GIT_ASKPASS_NODE
	# ENV VSCODE_GIT_IPC_HANDLE
	# ENV VSCODE_IPC_HOOK_CLI

	# if CI or vscode then force submodule update?

	if(DEFINED ENV{VSCODE_IPC_HOOK_CLI})
		#message(STATUS "VSCODE_IPC_HOOK_CLI: $ENV{VSCODE_IPC_HOOK_CLI}")

		#message(STATUS "TERM_PROGRAM: $ENV{TERM_PROGRAM}")

		#message(STATUS "VSCODE_GIT_ASKPASS_MAIN: $ENV{VSCODE_GIT_ASKPASS_MAIN}")
		#message(STATUS "VSCODE_GIT_ASKPASS_NODE: $ENV{VSCODE_GIT_ASKPASS_NODE}")
		#message(STATUS "VSCODE_GIT_IPC_HANDLE: $ENV{VSCODE_GIT_IPC_HANDLE}")
	endif()

	if(GIT_FOUND AND GIT_SUBMODULE AND NOT DEFINED ENV{GIT_SUBMODULES_ARE_EVIL})

		set(REL_PATH)

		if(IS_ABSOLUTE ${PATH})
			file(RELATIVE_PATH REL_PATH ${PX4_SOURCE_DIR} ${PATH})
		else()
			file(RELATIVE_PATH REL_PATH ${PX4_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${PATH})
		endif()

		if(NOT EXISTS ${PATH}/.git)
			execute_process(
				COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive --jobs=4 -- ${REL_PATH}
				OUTPUT_VARIABLE GIT_SUBMODULE_UPDATE
				RESULT_VARIABLE GIT_SUBMODULE_UPDATE_RESULT
				WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			)
			if(NOT GIT_SUBMODULE_UPDATE_RESULT EQUAL "0")
				message(FATAL_ERROR "git submodule update --init --recursive -- ${REL_PATH} failed: ${GIT_SUBMODULE_UPDATE}")
			endif()

		else()
			execute_process(
				COMMAND ${GIT_EXECUTABLE} submodule status -- ${REL_PATH}
				OUTPUT_VARIABLE GIT_SUBMODULE_STATUS
				RESULT_VARIABLE GIT_SUBMODULE_STATUS_RESULT
				WORKING_DIRECTORY ${PX4_SOURCE_DIR}
			)
			#message(STATUS "git submodule status ${REL_PATH} result ${GIT_SUBMODULE_STATUS_RESULT} : ${GIT_SUBMODULE_STATUS}")

			if(NOT GIT_SUBMODULE_STATUS_RESULT EQUAL "0")
				message(FATAL_ERROR "git submodule status -- ${REL_PATH} failed: ${GIT_SUBMODULE_STATUS}")
			else()
				# submodule status characters
				string(FIND "${GIT_SUBMODULE_STATUS}" "+" submodule_plus)
				string(FIND "${GIT_SUBMODULE_STATUS}" "-" submodule_minus)
				string(FIND "${GIT_SUBMODULE_STATUS}" "U" submodule_merge_conflicts)

				# + commit does not match the SHA-1 found in the index
				if("${submodule_plus}" EQUAL 0)
					# GIT_SHA_HEAD: submodule git SHA on current HEAD
					execute_process(
						COMMAND ${GIT_EXECUTABLE} ls-tree -d HEAD ${REL_PATH} | awk '{print $3;}
						OUTPUT_VARIABLE GIT_SHA_HEAD
						WORKING_DIRECTORY ${PX4_SOURCE_DIR}
					)

					# GIT_SHA_PREV_HEAD: submodule git SHA on previous HEAD
					execute_process(
						COMMAND ${GIT_EXECUTABLE} ls-tree -d HEAD@{1} ${REL_PATH} | awk '{print $3;}
						OUTPUT_VARIABLE GIT_SHA_PREV_HEAD
						WORKING_DIRECTORY ${PX4_SOURCE_DIR}
					)

					#message(STATUS "GIT_SHA_HEAD: ${GIT_SHA_HEAD}")
					#message(STATUS "GIT_SHA_PREV_HEAD: ${GIT_SHA_PREV_HEAD}")

					# TODO: check if submodule is dirty?

					if(${GIT_SHA_HEAD} MATCHES ${GIT_SHA_PREV_HEAD})
						#message(STATUS "${rel_path} GIT SHA matched previous")
						# submodule hasn't changed, assume it's safe to update
						execute_process(
							COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive --jobs=4 -- ${REL_PATH}
							OUTPUT_VARIABLE GIT_SUBMODULE_UPDATE
							WORKING_DIRECTORY ${PX4_SOURCE_DIR}
						)
					else()
						message(NOTICE "submodule ${REL_PATH} commit does not match the SHA-1 found in the index")
					endif()
				endif()

				# - uninitialized
				if("${submodule_minus}" EQUAL 0)
					execute_process(
						COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive --jobs=4 -- ${REL_PATH}
						OUTPUT_VARIABLE GIT_SUBMODULE_UPDATE
						WORKING_DIRECTORY ${PX4_SOURCE_DIR}
					)
				endif()

				# U merge conflicts
				if("${submodule_merge_conflicts}" EQUAL 0)
					message(NOTICE "submodule ${REL_PATH} has merge conflicts")
				endif()
			endif()
		endif()


		add_custom_target(${TARGET})

		set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PATH}/.git)

	endif()
endfunction()
