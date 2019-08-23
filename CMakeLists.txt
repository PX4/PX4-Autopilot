############################################################################
#
# Copyright (c) 2017 - 2019 PX4 Development Team. All rights reserved.
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
# CMAKE CODING STANDARD FOR PX4
#
# Structure
# ---------------------------------------------------------------------------
#
# * Common functions should be included in px_base.cmake.
#
# * OS/ board specific fucntions should be include in
#	px_impl_${PX4_PLATFORM}.cmake or px4_impl_${PX4_PLATFORM}_${PX4_BOARD}.cmake.
#
# Formatting
# ---------------------------------------------------------------------------
#
# * Use hard indents to match the px4 source code.
#
# * All function and script arguments are upper case.
#
# * All local variables are lower case.
#
# * All cmake functions are lowercase.
#
# * For else, endif, endfunction, etc, never put the name of the statement
#
# Functions/Macros
# ---------------------------------------------------------------------------
#
# * Use px4_parse_function_args to parse functions and check for required
#   arguments. Unless there is only one argument in the function and it is clear.
#
# * Never use macros. They allow overwriting global variables and this
#	makes variable declarations hard to locate.
#
# * If a target from add_custom_* is set in a function, explicitly pass it
#	as an output argument so that the target name is clear to the user.
#
# * Avoid use of global variables in functions. Functions in a nested
#	scope may use global variables, but this makes it difficult to
#	reuse functions.
#
# Included CMake Files
# ---------------------------------------------------------------------------
#
# * All variables in config files must have the prefix "config_".
#
# * Never set global variables in an included cmake file,
#	you may only define functions. This excludes config and Toolchain files.
#	This makes it clear to the user when variables are being set or targets
#	are being created.
#
# * Setting a global variable in a CMakeLists.txt file is ok, because
#	each CMakeLists.txt file has scope in the current directory and all
#	subdirectories, so it is not truly global.
#
# * All toolchain files should be included in the cmake
#	directory and named Toolchain-"name".cmake.
#
# Misc
# ---------------------------------------------------------------------------
#
# * If referencing a string variable, don't put it in quotes.
#	Don't do "${PX4_PLATFORM}" STREQUAL "posix",
#	instead type ${PX4_PLATFORM} STREQUAL "posix". This will throw an
#	error when ${PX4_PLATFORM} is not defined instead of silently
#	evaluating to false.
#
#=============================================================================

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake)

#=============================================================================
# git
#
include(px4_git)

execute_process(
	COMMAND git describe --always --tags
	OUTPUT_VARIABLE PX4_GIT_TAG
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

define_property(GLOBAL PROPERTY PX4_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 module libs"
                 FULL_DOCS "List of all PX4 module libraries"
                 )

define_property(GLOBAL PROPERTY PX4_MODULE_PATHS
                 BRIEF_DOCS "PX4 module paths"
                 FULL_DOCS "List of paths to all PX4 modules"
                 )

#=============================================================================
# configuration
#

set(CONFIG "px4_sitl_default" CACHE STRING "desired configuration")

include(px4_add_module)
set(config_module_list)
set(config_df_driver_list)

# find PX4 config
#  look for in tree board config that matches CONFIG input
if(NOT PX4_CONFIG_FILE)

	file(GLOB_RECURSE board_configs
		RELATIVE "${PX4_SOURCE_DIR}/boards"
		"boards/*.cmake"
		)

	set(PX4_CONFIGS ${board_configs} CACHE STRING "PX4 board configs" FORCE)

	foreach(filename ${board_configs})
		# parse input CONFIG into components to match with existing in tree configs
		#  the platform prefix (eg nuttx_) is historical, and removed if present
		string(REPLACE ".cmake" "" filename_stripped ${filename})
		string(REPLACE "/" ";" config ${filename_stripped})
		list(LENGTH config config_len)

		if(${config_len} EQUAL 3)


			list(GET config 0 vendor)
			list(GET config 1 model)
			list(GET config 2 label)

			set(board "${vendor}${model}")

			# <VENDOR>_<MODEL>_<LABEL> (eg px4_fmu-v2_default)
			# <VENDOR>_<MODEL>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${vendor}_${model}_${label}") OR # match full vendor, model, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${vendor}_${model}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()

			# <BOARD>_<LABEL> (eg px4_fmu-v2_default)
			# <BOARD>_default (eg px4_fmu-v2) # allow skipping label if "default"
			if ((${CONFIG} MATCHES "${board}_${label}") OR # match full board, label
			    ((${label} STREQUAL "default") AND (${CONFIG} STREQUAL "${board}")) # default label can be omitted
			)
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()


			# LEGACY form
			# <OS>_<BOARD>_<LABEL> (eg nuttx_px4_fmu-v2_default)
			string(REGEX REPLACE "^nuttx_|^posix_|^qurt_" "" config_no_os ${CONFIG}) # ignore OS prefix

			if ((${config_no_os} MATCHES "${board}_${label}"))
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()


			# LEGACY form special case to ease board layout transition (2018-11-18)
			#  match board with model and label only: eg sitl_default -> px4_sitl_default
			if ((${config_no_os} MATCHES "${model}_${label}"))
				set(PX4_CONFIG_FILE "${PX4_SOURCE_DIR}/boards/${filename}" CACHE FILEPATH "path to PX4 CONFIG file" FORCE)
				break()
			endif()

		endif()

	endforeach()
endif()

if(NOT PX4_CONFIG_FILE)
	message(FATAL_ERROR "PX4 config file not set, try one of ${PX4_CONFIGS}")
endif()

message(STATUS "PX4 config file: ${PX4_CONFIG_FILE}")
include(px4_add_board)
include(${PX4_CONFIG_FILE})
message(STATUS "PX4 config: ${PX4_CONFIG}")
message(STATUS "PX4 platform: ${PX4_PLATFORM}")

if (ENABLE_LOCKSTEP_SCHEDULER)
	add_definitions(-DENABLE_LOCKSTEP_SCHEDULER)
	message(STATUS "PX4 lockstep: enabled")
else()
	message(STATUS "PX4 lockstep: disabled")
endif()

# external modules
set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

if (NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	get_filename_component(EXTERNAL_MODULES_LOCATION "${EXTERNAL_MODULES_LOCATION}" ABSOLUTE)
endif()

set_property(GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)

include(platforms/${PX4_PLATFORM}/cmake/px4_impl_os.cmake)
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/init.cmake")
	include(init)
endif()

# CMake build type (Debug Release RelWithDebInfo MinSizeRel Coverage)
if (NOT CMAKE_BUILD_TYPE)
	if (${PX4_PLATFORM} STREQUAL "nuttx")
		set(PX4_BUILD_TYPE "MinSizeRel")
	else()
		set(PX4_BUILD_TYPE "RelWithDebInfo")
	endif()

	set(CMAKE_BUILD_TYPE ${PX4_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")

#=============================================================================

message(STATUS "PX4 version: ${PX4_GIT_TAG}")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

#=============================================================================
# project definition
#
project(px4 CXX C ASM)

set(package-contact "px4users@googlegroups.com")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# For the catkin build process, unset build of dynamically-linked binaries
# and do not change CMAKE_RUNTIME_OUTPUT_DIRECTORY
if (NOT CATKIN_DEVEL_PREFIX)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PX4_BINARY_DIR})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX4_BINARY_DIR})
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX4_BINARY_DIR})
else()
	SET(BUILD_SHARED_LIBS OFF)
endif()

#=============================================================================

# Setup install paths
if (${PX4_PLATFORM} STREQUAL "posix")

	# This makes it possible to dynamically load code which depends on symbols
	# inside the px4 executable.
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
	set(CMAKE_ENABLE_EXPORTS ON)

	include(coverage)
	include(sanitizers)

	# Define GNU standard installation directories
	include(GNUInstallDirs)

	if (NOT CMAKE_INSTALL_PREFIX)
		set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
	endif()
endif()

#=============================================================================
# ccache
#
option(CCACHE "Use ccache if available" ON)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE AND CCACHE_PROGRAM AND NOT DEFINED ENV{CCACHE_DISABLE})

	get_filename_component(ccache_real_path ${CCACHE_PROGRAM} REALPATH)
	get_filename_component(cxx_real_path ${CMAKE_CXX_COMPILER} REALPATH)
	get_filename_component(cxx_abs_path ${CMAKE_CXX_COMPILER} ABSOLUTE)

	if ("${ccache_real_path}" STREQUAL "${cxx_real_path}")
		message(STATUS "ccache enabled via symlink (${cxx_abs_path} -> ${cxx_real_path})")
	else()
		message(STATUS "ccache enabled (export CCACHE_DISABLE=1 to disable)")
		set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
	endif()

endif()

#=============================================================================
# find programs and packages
#

# see if catkin was invoked to build this
if (CATKIN_DEVEL_PREFIX)
	message(STATUS "catkin ENABLED")
	find_package(catkin REQUIRED)
	if (catkin_FOUND)
		catkin_package()
	else()
		message(FATAL_ERROR "catkin not found")
	endif()
endif()

find_package(PythonInterp REQUIRED)

option(PYTHON_COVERAGE "Python code coverage" OFF)
if(PYTHON_COVERAGE)
	message(STATUS "python coverage enabled")
	set(PYTHON_EXECUTABLE coverage run -p)
else()
	# run normally (broken under coveragepy)
	px4_find_python_module(jinja2 REQUIRED)
endif()

#=============================================================================
# build flags
#
include(px4_add_common_flags)
px4_add_common_flags()
px4_os_add_flags()

#=============================================================================
# message, and airframe generation
#
include(px4_metadata)

add_subdirectory(msg EXCLUDE_FROM_ALL)

px4_generate_airframes_xml(BOARD ${PX4_BOARD})

#=============================================================================
# DriverFramework
#
px4_add_git_submodule(TARGET git_driverframework PATH "src/lib/DriverFramework")
set(OS ${PX4_PLATFORM})
add_subdirectory(src/lib/DriverFramework/framework)

# List the DriverFramework drivers
if (DEFINED config_df_driver_list)
	message("DF Drivers: ${config_df_driver_list}")
endif()

set(df_driver_libs)
foreach(driver ${config_df_driver_list})
	add_subdirectory(src/lib/DriverFramework/drivers/${driver})
	list(APPEND df_driver_libs df_${driver})
	message("Adding DF driver: ${driver}")
endforeach()

#=============================================================================
# external projects
#
set(ep_base ${PX4_BINARY_DIR}/external)
set_property(DIRECTORY PROPERTY EP_BASE ${ep_base})

# add external project install folders to build
link_directories(${ep_base}/Install/lib)
include_directories(${ep_base}/Install/include)
# add the directories so cmake won't warn
execute_process(COMMAND cmake -E make_directory ${ep_base}/Install/lib)
execute_process(COMMAND cmake -E make_directory ${ep_base}/Install/include)

#=============================================================================
# external modules
#
set(external_module_paths)
if (NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	message(STATUS "External modules: ${EXTERNAL_MODULES_LOCATION}")
	add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src" external_modules)

	foreach(external_module ${config_module_list_external})
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/src/${external_module} external_modules/${external_module})
		list(APPEND external_module_paths ${EXTERNAL_MODULES_LOCATION}/src/${external_module})
	endforeach()
endif()

#=============================================================================
# Testing - Automatic unit and integration testing with CTest
#

# optionally enable cmake testing (supported only on posix)
option(CMAKE_TESTING "Configure test targets" OFF)
if (${PX4_CONFIG} STREQUAL "px4_sitl_test")
	set(CMAKE_TESTING ON)
endif()
if(CMAKE_TESTING)
	include(CTest) # sets BUILD_TESTING variable
endif()

# enable test filtering to run only specific tests with the ctest -R regex functionality
set(TESTFILTER "" CACHE STRING "Filter string for ctest to selectively only run specific tests (ctest -R)")

# if testing is enabled download and configure gtest
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake/gtest/)
include(px4_add_gtest)
if(BUILD_TESTING)
	include(gtest)
endif()

add_custom_target(test_results
		COMMAND GTEST_COLOR=1 ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test -R ${TESTFILTER} USES_TERMINAL
		DEPENDS
			px4
			examples__dyn_hello
			test_mixer_multirotor
		USES_TERMINAL
		COMMENT "Running tests"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results PROPERTIES EXCLUDE_FROM_ALL TRUE)

#=============================================================================
# subdirectories
#
add_library(parameters_interface INTERFACE)

include(px4_add_library)
add_subdirectory(src/lib EXCLUDE_FROM_ALL)

add_subdirectory(platforms/${PX4_PLATFORM}/src/px4)
add_subdirectory(src/platforms EXCLUDE_FROM_ALL)
add_subdirectory(src/modules/uORB EXCLUDE_FROM_ALL) # TODO: platform layer
add_subdirectory(src/drivers/boards EXCLUDE_FROM_ALL)

if(EXISTS "${PX4_BOARD_DIR}/CMakeLists.txt")
	add_subdirectory(${PX4_BOARD_DIR})
endif()

foreach(module ${config_module_list})
	add_subdirectory(src/${module})
endforeach()

# must be the last module before firmware
add_subdirectory(src/lib/parameters EXCLUDE_FROM_ALL)
target_link_libraries(parameters_interface INTERFACE parameters)

# firmware added last to generate the builtin for included modules
add_subdirectory(platforms/${PX4_PLATFORM})

#=============================================================================
# uORB graph generation: add a custom target 'uorb_graph'
#
set(uorb_graph_config ${PX4_BOARD})

set(graph_module_list "")
foreach(module ${config_module_list})
	set(graph_module_list "${graph_module_list}" "--src-path" "src/${module}")
endforeach()

add_custom_command(OUTPUT ${uorb_graph_config}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/uorb_graph/create.py
		${module_list}
		--exclude-path src/examples
		--file ${PX4_SOURCE_DIR}/Tools/uorb_graph/graph_${uorb_graph_config}
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	COMMENT "Generating uORB graph"
)
add_custom_target(uorb_graph DEPENDS ${uorb_graph_config})

#=============================================================================
# Doxygen
#
option(BUILD_DOXYGEN "Build doxygen documentation" OFF)

if (BUILD_DOXYGEN)
	find_package(Doxygen)
	if (DOXYGEN_FOUND)
	    # set input and output files
	    set(DOXYGEN_IN ${CMAKE_SOURCE_DIR}/Documentation/Doxyfile.in)
	    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

	    # request to configure the file
	    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

	    # note the option ALL which allows to build the docs together with the application
	    add_custom_target(doxygen ALL
	        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
	        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
			COMMENT "Generating documentation with Doxygen"
			DEPENDS uorb_msgs parameters
			VERBATIM
			USES_TERMINAL
			)

	else()
		message("Doxygen needs to be installed to generate documentation")
	endif()
endif()

#=============================================================================
# Metadata - helpers for generating documentation
#

add_custom_target(metadata_airframes
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/docs
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		-v -a ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d
		--markdown ${PX4_BINARY_DIR}/docs/airframes.md
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_airframes.py
		-v -a ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d
		--xml ${PX4_BINARY_DIR}/docs/airframes.xml
	COMMENT "Generating full airframe metadata (markdown and xml)"
	USES_TERMINAL
)

file(GLOB_RECURSE yaml_config_files ${PX4_SOURCE_DIR}/src/modules/*.yaml
	${PX4_SOURCE_DIR}/src/drivers/*.yaml ${PX4_SOURCE_DIR}/src/lib/*.yaml)
add_custom_target(metadata_parameters
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/docs
	COMMAND ${PYTHON_EXECUTABLE}
	${PX4_SOURCE_DIR}/Tools/serial/generate_config.py --all-ports --params-file ${PX4_SOURCE_DIR}/src/generated_serial_params.c --config-files ${yaml_config_files}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/parameters/px_process_params.py
		--src-path `find ${PX4_SOURCE_DIR}/src -maxdepth 4 -type d`
		--inject-xml ${PX4_SOURCE_DIR}/src/lib/parameters/parameters_injected.xml
		--markdown ${PX4_BINARY_DIR}/docs/parameters.md
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/parameters/px_process_params.py
		--src-path `find ${PX4_SOURCE_DIR}/src -maxdepth 4 -type d`
		--inject-xml ${PX4_SOURCE_DIR}/src/lib/parameters/parameters_injected.xml
		--xml ${PX4_BINARY_DIR}/docs/parameters.xml
	COMMENT "Generating full parameter metadata (markdown and xml)"
	USES_TERMINAL
)

add_custom_target(metadata_module_documentation
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/docs
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/px_process_module_doc.py -v --src-path ${PX4_SOURCE_DIR}/src
		--markdown ${PX4_BINARY_DIR}/docs/modules
	COMMENT "Generating module documentation"
	USES_TERMINAL
)

add_custom_target(all_metadata
	DEPENDS
		metadata_airframes
		metadata_parameters
		metadata_module_documentation
)

#=============================================================================
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${PX4_CONFIG})
set(CPACK_PACKAGE_VERSION ${PX4_GIT_TAG})
set(CPACK_PACKAGE_CONTACT ${package-contact})
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF) # TODO: review packaging for linux boards
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "The PX4 Pro autopilot.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PX4_CONFIG}-${PX4_GIT_TAG}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PX4_GIT_TAG}")
set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")
set(CPACK_PACKAGING_INSTALL_PREFIX "")
set(CPACK_SET_DESTDIR "OFF")

if ("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(CPACK_GENERATOR "TBZ2")
	find_program(DPKG_PROGRAM dpkg)
	if (EXISTS ${DPKG_PROGRAM})
		list (APPEND CPACK_GENERATOR "DEB")
	endif()
else()
	set(CPACK_GENERATOR "ZIP")
endif()

include(CPack)
include(bloaty)
