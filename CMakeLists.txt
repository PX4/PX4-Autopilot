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

#=============================================================================
# CMAKE CODING STANDARD FOR PX4
#
# Structure
# ---------------------------------------------------------------------------
#
# * Common functions should be included in px_base.cmake.
#
# * OS/ board specific fucntions should be include in
#	px_impl_${OS}.cmake or px4_impl_${OS}_${BOARD}.cmake.
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
#	Instead of the very confusing:
#		if (${var} STREQUAL "1") <-- condition now becomes if name
#			# do somthing
#		elseif (${var} STREQUAL "2") <-- another condition
#			# do somthing
#		else (${var} STREQUAL "1") <-- tag is referring to name of if
#			# do somthing
#		endif (${var} STREQUAL "1") <-- tag is referring to name of if
#
#	Do this:
#		if (${var} STREQUAL "1") <-- condition now becomes if name
#			# do somthing
#		elseif (${var} STREQUAL "2") <-- another condition
#			# do somthing
#		else () <-- leave blank
#			# do somthing
#		endif () <-- leave blank
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
#	resuse functions.
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
#	subdirecties, so it is not truly global.
#
# * All toolchain files should be included in the cmake
#	directory and named Toolchain-"name".cmake.
#
# Misc
# ---------------------------------------------------------------------------
#
# * If referencing a string variable, don't put it in quotes.
#	Don't do "${OS}" STREQUAL "posix",
#	instead type ${OS} STREQUAL "posix". This will throw an
#	error when ${OS} is not defined instead of silently
#	evaluating to false.
#
#=============================================================================

if("${CMAKE_VERSION}" VERSION_LESS 3.1.0)
	message("Not a valid CMake version")
	message("On Ubuntu >= 16.04, install or upgrade via:")
	message("	sudo apt-get install cmake")
	message("")
	message("Official website: https://cmake.org/download/")
	message(FATAL_ERROR "Update CMake and try again" )
endif()

# Warning: Changing this modifies CMake's internal workings
# and leads to wrong toolchain detection
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

execute_process(
	COMMAND Tools/check_submodules.sh
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

#=============================================================================
# parameters
#

set(CMAKE_BUILD_TYPE "" CACHE STRING "build type")
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
	STRINGS ";Debug;Release;RelWithDebInfo;MinSizeRel")
set(CONFIG "posix_sitl_default" CACHE STRING "desired configuration")

file(GLOB_RECURSE configs RELATIVE cmake/configs "cmake/configs/*.cmake")
set_property(CACHE CONFIG PROPERTY STRINGS ${configs})

set(THREADS "4" CACHE STRING "number of threads to use for external build processes")
set(DEBUG_PORT "/dev/ttyACM0" CACHE STRING "debugging port")
set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	get_filename_component(EXTERNAL_MODULES_LOCATION "${EXTERNAL_MODULES_LOCATION}" ABSOLUTE)
endif()

#=============================================================================
# configuration
#
# must come before project to set toolchain

string(REPLACE "_" ";" config_args ${CONFIG})
list(GET config_args 0 OS)
list(GET config_args 1 BOARD)
list(GET config_args 2 LABEL)
set(target_name "${OS}-${BOARD}-${LABEL}")

# version info from git
execute_process(
	COMMAND Tools/tag_to_version.py --root ${PX4_SOURCE_DIR}
	OUTPUT_VARIABLE version
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)
execute_process(
	COMMAND git describe --always --tags
	OUTPUT_VARIABLE git_tag
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)
message(STATUS "PX4 VERSION: ${git_tag}")
message(STATUS "CONFIG: ${target_name}")

# The URL for the elf file for crash logging
if (DEFINED ENV{BUILD_URI})
	set(BUILD_URI $ENV{BUILD_URI})
else()
	set(BUILD_URI "localhost")
endif()

add_definitions(-DBUILD_URI=${BUILD_URI})

# Define GNU standard installation directories
include(GNUInstallDirs)

# Add support for external project building
include(ExternalProject)

# Setup install paths
if(NOT CMAKE_INSTALL_PREFIX)
	if (${OS} STREQUAL "posix")
		set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
	endif()
endif()
if (CMAKE_INSTALL_PREFIX)
	message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
endif()

list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
set(config_module "configs/${CONFIG}")
include(${config_module})

# cmake modules
include(ExternalProject)

# require px4 module interface
set(px4_required_interface
	px4_os_prebuild_targets
	px4_os_add_flags
	)
foreach(cmd ${px4_required_interface})
	if(NOT COMMAND ${cmd})
		message(FATAL_ERROR "${config_module} must implement ${cmd}")
	endif()
endforeach()
set(px4_required_config
	config_module_list
	)
foreach(conf ${px4_required_config})
	if(NOT DEFINED ${conf})
		message(FATAL_ERROR "cmake/${config_module} must define ${conf}")
	endif()
endforeach()

# List the DriverFramework drivers
if(DEFINED config_df_driver_list)
	message("DF Drivers: ${config_df_driver_list}")
endif()

# force static lib build
set(BUILD_SHARED_LIBS OFF)

#=============================================================================
# ccache
#
option(CCACHE "Use ccache if available" OFF)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE AND CCACHE_PROGRAM)
	message(STATUS "Enabled ccache: ${CCACHE_PROGRAM}")
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

#=============================================================================
# project definition
#
project(px4 CXX C ASM)

set(package-contact "px4users@googlegroups.com")

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
px4_find_python_module(jinja2 REQUIRED)

#=============================================================================
# cmake testing
#
enable_testing()
include(CTest)

#=============================================================================
# generate compile command database
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#=============================================================================
# check required toolchain variables
#

# PX4 requires c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PX4 requires c99
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(required_variables
	CMAKE_C_COMPILER_ID
	)
foreach(var ${required_variables})
	if (NOT ${var})
		message(FATAL_ERROR "Toolchain/config must define ${var}")
	endif()
endforeach()

# print full c compiler version
execute_process(COMMAND ${CMAKE_C_COMPILER} --version
		OUTPUT_VARIABLE c_compiler_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
STRING(REGEX MATCH "[^\n]*" c_compiler_version_short ${c_compiler_version})
message(STATUS "C compiler: ${c_compiler_version_short}")

# print full c++ compiler version
execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version
		OUTPUT_VARIABLE cxx_compiler_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
STRING(REGEX MATCH "[^\n]*" cxx_compiler_version_short ${cxx_compiler_version})
message(STATUS "C++ compiler: ${cxx_compiler_version_short}")


#=============================================================================
# git
#
px4_add_git_submodule(TARGET git_cmake_hexagon PATH "cmake/cmake_hexagon")
px4_add_git_submodule(TARGET git_driverframework PATH "src/lib/DriverFramework")
px4_add_git_submodule(TARGET git_ecl PATH "src/lib/ecl")
px4_add_git_submodule(TARGET git_gazebo PATH "Tools/sitl_gazebo")
px4_add_git_submodule(TARGET git_gencpp PATH "Tools/gencpp")
px4_add_git_submodule(TARGET git_genmsg PATH "Tools/genmsg")
px4_add_git_submodule(TARGET git_gtest PATH "unittests/gtest")
px4_add_git_submodule(TARGET git_jmavsim PATH "Tools/jMAVSim")
px4_add_git_submodule(TARGET git_matrix PATH "src/lib/matrix")
px4_add_git_submodule(TARGET git_mavlink PATH "mavlink/include/mavlink/v1.0")
px4_add_git_submodule(TARGET git_mavlink2 PATH "mavlink/include/mavlink/v2.0")
px4_add_git_submodule(TARGET git_nuttx PATH "NuttX")
px4_add_git_submodule(TARGET git_uavcan PATH "src/modules/uavcan/libuavcan")

add_custom_target(submodule_clean
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	COMMAND git submodule deinit -f .
	COMMAND rm -rf .git/modules/*
	)

#=============================================================================
# misc targets
#
add_custom_target(check_format
	COMMAND Tools/check_code_style.sh
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)

add_custom_target(config
	COMMAND cmake-gui .
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)

#=============================================================================
# external libraries
#
px4_os_prebuild_targets(OUT prebuild_targets
	BOARD ${BOARD}
	THREADS ${THREADS})

#=============================================================================
# build flags
#
px4_os_add_flags(
	BOARD ${BOARD}
	C_FLAGS c_flags
	CXX_FLAGS cxx_flags
	OPTIMIZATION_FLAGS optimization_flags
	EXE_LINKER_FLAGS exe_linker_flags
	INCLUDE_DIRS include_dirs
	LINK_DIRS link_dirs
	DEFINITIONS definitions)

px4_join(OUT CMAKE_EXE_LINKER_FLAGS LIST "${exe_linker_flags}" GLUE " ")
px4_join(OUT CMAKE_C_FLAGS LIST "${c_flags};${optimization_flags}" GLUE " ")
px4_join(OUT CMAKE_CXX_FLAGS LIST "${cxx_flags};${optimization_flags}" GLUE " ")

include_directories(${include_dirs})
#message("INCLUDE_DIRS=${include_dirs}")
link_directories(${link_dirs})
add_definitions(${definitions})

#=============================================================================
# source code generation
#

add_subdirectory(msg)
px4_generate_messages(TARGET msg_gen
	MSG_FILES ${msg_files}
	OS ${OS}
	INCLUDES ${msg_include_paths}
	DEPENDS git_genmsg git_gencpp prebuild_targets
	)
px4_generate_parameters_xml(OUT parameters.xml 
	BOARD ${BOARD}
	MODULES ${config_module_list}
	OVERRIDES ${PARAM_DEFAULT_OVERRIDES})
px4_generate_airframes_xml(OUT airframes.xml BOARD ${BOARD})
add_custom_target(xml_gen
	DEPENDS parameters.xml airframes.xml)

if(NOT "${config_nuttx_config}" STREQUAL "bootloader")
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/airframes.xml
	${CMAKE_CURRENT_BINARY_DIR}/parameters.xml
	DESTINATION .)
endif()

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
# DriverFramework Drivers
#
set(df_driver_libs)
foreach(driver ${config_df_driver_list})
	add_subdirectory(src/lib/DriverFramework/drivers/${driver})
	list(APPEND df_driver_libs df_${driver})
	message("Adding DF driver: ${driver}")
endforeach()

#=============================================================================
# external modules
#
if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	message(STATUS "External modules: ${EXTERNAL_MODULES_LOCATION}")
	add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src" external_modules_src)

	set(config_module_list_external_expanded)
	foreach(external_module ${config_module_list_external})
		list(APPEND config_module_list_external_expanded
			${EXTERNAL_MODULES_LOCATION}/src/${external_module})
	endforeach()
	set(config_module_list
		${config_module_list}
		${config_module_list_external_expanded}
		)
endif()

#=============================================================================
# subdirectories
#
set(module_libraries)
foreach(module ${config_module_list})
	string(REGEX MATCH "^[./]" external_module ${module})
	if(external_module)
		STRING(REGEX REPLACE "//" "/" EXT_MODULE ${module})
		STRING(REGEX REPLACE "/" "__" EXT_MODULE_PREFIX ${EXT_MODULE})
		add_subdirectory(${module} ${PX4_BINARY_DIR}/${EXT_MODULE_PREFIX})
	else()
		add_subdirectory(src/${module})
	endif()
	px4_mangle_name(${module} mangled_name)
	list(APPEND module_libraries ${mangled_name})
	#message(STATUS "adding module: ${module}")
endforeach()

# Keep track of external shared libs required for modules
set(module_external_libraries "${module_external_libraries}" CACHE INTERNAL "module_external_libraries")

add_subdirectory(src/firmware/${OS})

#add_dependencies(df_driver_framework nuttx_export_${CONFIG})
if (NOT "${OS}" STREQUAL "nuttx")
endif()

if (config_io_board)
	add_subdirectory(src/modules/px4iofirmware)
endif()

#=============================================================================
# generate custom target to print for all executable and module cmake targets
#
if(all_posix_cmake_targets)
	list(SORT all_posix_cmake_targets)
	px4_join(OUT posix_cmake_target_list LIST ${all_posix_cmake_targets} GLUE "\\n")
	add_custom_target(list_cmake_targets
		COMMAND sh -c "printf \"${posix_cmake_target_list}\\n\""
		COMMENT "List of cmake targets that can be matched by PX4_NO_OPTIMIZATION:"
		VERBATIM
		)
endif()

#=============================================================================
# generate git version
#
px4_create_git_hash_header()

#=============================================================================
# packaging
#
# Important to having packaging at end of cmake file.
#
set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${CONFIG})
set(CPACK_PACKAGE_VERSION ${version})
set(CPACK_PACKAGE_CONTACT ${package-contact})
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(short-description "The px4 autopilot.")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION ${short-description})
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CONFIG}-${git_tag}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${version}")
set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")
set(CPACK_PACKAGING_INSTALL_PREFIX "")
set(CPACK_SET_DESTDIR "OFF")
if ("${CMAKE_SYSTEM}" MATCHES "Linux")
	find_program(DPKG_PROGRAM dpkg)
	if (EXISTS ${DPKG_PROGRAM})
		list (APPEND CPACK_GENERATOR "DEB")
	endif()
endif()
include(CPack)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
