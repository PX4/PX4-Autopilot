############################################################################
#
# Copyright (c) 2017 - 2024 PX4 Development Team. All rights reserved.
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
#	px_impl_${PX4_PLATFORM}.cmake
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

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Extract the absolute directory to the root of the PX4 workspace
set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "PX4 source directory" FORCE)

# Extract the absolute directory to the build file of the target
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE FILEPATH "PX4 binary directory" FORCE)

# Append the directory to the cmake functions (Will be included throughout build chain)
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake)

# Run cmake parse_functions script 
# ${CMAKE_CURRENT_SOURCE_DIR}/cmake/px4_parse_function_args.cmake
include(px4_parse_function_args)

#=============================================================================
# GIT
#=============================================================================

# Run cmake git script (${CMAKE_CURRENT_SOURCE_DIR}/cmake/px4_git.cmake)
# This includes a function definition to add submodules from git.
include(px4_git)

# Extract the version of PX4 being used from the git tag (Stored in PX4_GIT_TAG)
execute_process(
	COMMAND git describe --exclude ext/* --tags --match "v[0-9]*"
	OUTPUT_VARIABLE PX4_GIT_TAG
	OUTPUT_STRIP_TRAILING_WHITESPACE
	RESULTS_VARIABLE GIT_DESCRIBE_RESULT
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)
    
# If no proper git tag found, default to v0.0.0
if(NOT ${GIT_DESCRIBE_RESULT} MATCHES "0")
	set(PX4_GIT_TAG "v0.0.0")
endif()

# Output Git Tag
message(STATUS "PX4_GIT_TAG: ${PX4_GIT_TAG}")

# Store git version as list seperated by ';' and store in variable VERSION_LIST
string(REPLACE "." ";" VERSION_LIST ${PX4_GIT_TAG})

# Find the major version of PX4 from the version list
list(GET VERSION_LIST 0 PX4_VERSION_MAJOR)
string(REPLACE "v" "" PX4_VERSION_MAJOR ${PX4_VERSION_MAJOR})

# Find the minor version of PX4 from the version list
list(GET VERSION_LIST 1 PX4_VERSION_MINOR)

# Find the patch version of PX4 from the version list
list(GET VERSION_LIST 2 PX4_VERSION_PATCH)
string(REPLACE "-" ";" PX4_VERSION_PATCH ${PX4_VERSION_PATCH})
list(GET PX4_VERSION_PATCH 0 PX4_VERSION_PATCH)

# Extract the hash version of PX4 branch (Capture only the hash part after 'g')
string(REGEX MATCH "g([a-f0-9]+)$" GIT_HASH "${PX4_GIT_TAG}")
set(PX4_GIT_HASH ${CMAKE_MATCH_1})

# Track all PX4 module libraries in project
define_property(GLOBAL PROPERTY PX4_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 module libs"
                 FULL_DOCS "List of all PX4 module libraries"
                 )

# Track all PX4 kernel-side module libraries             
define_property(GLOBAL PROPERTY PX4_KERNEL_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 kernel side module libs"
                 FULL_DOCS "List of all PX4 kernel module libraries"
                 )

# Track paths to each PX4 module
define_property(GLOBAL PROPERTY PX4_MODULE_PATHS
                 BRIEF_DOCS "PX4 module paths"
                 FULL_DOCS "List of paths to all PX4 modules"
                 )

# Track all source files from PX4 modules and libraries
define_property(GLOBAL PROPERTY PX4_SRC_FILES
                 BRIEF_DOCS "src files from all PX4 modules & libs"
                 FULL_DOCS "SRC files from px4_add_{module,library}"
                 )

#=============================================================================
# CONFIGURATION
#=============================================================================

# Include function to add PX4 module to build chain
# ${CMAKE_CURRENT_SOURCE_DIR}/cmake/px4_add_module.cmake
include(px4_add_module)
set(config_module_list)
set(config_kernel_list)

# Find Python3
find_package(PythonInterp 3)

# Output error message to tell users how to install python3 if python not found
if(NOT PYTHONINTERP_FOUND)
	message(FATAL_ERROR "Python 3 not found. Please install Python 3:\n"
		"    Ubuntu: sudo apt install python3 python3-dev python3-pip\n"
		"    macOS: brew install python")
endif()

# Create an option to run python with `converge run -p` when enabled 
# (default is to run with off)
option(PYTHON_COVERAGE "Python code coverage" OFF)
if(PYTHON_COVERAGE)
	message(STATUS "python coverage enabled")
	set(PYTHON_EXECUTABLE coverage run -p)
endif()

# Set and find the appropriate PX4 board configuration for the build. 
# Based on CONFIG variable, and defines related board variables and paths 
# to use during compilation.
#
# Parsed argument into make is such that <vendor>_<model>_<label> 
#       exp: px4_sitl
#               vendor=px4
#               model=sitl
#               label=default
#
#       Note: if no label tag is provided it automacitally is set to default.
#
# px4_config extracts these tags and uses them to create a `boardconfig` 
# makefile which can be found at <PX4_BINARY_DIR>/boardconfig, which is then 
# used to create variables by the compiler in the boards directory: 
#
#       PX4_BOARD_DIR=<PX4_SOURCE_DIR>/boards/<vendor>/<model>/
#       PX4_CONFIG=<PX4_BOARD_DIR>/<label>.px4board
#
# The respective .px4board file is called with configuration parameters to be
# used by cmake. This file is located at.
#
# ${CMAKE_CURRENT_SOURCE_DIR}/cmake/px4_config.cmake
include(px4_config)

# Take the .px4board configuration parameters, construct variable names for them
# and then set their respective values to the parameter names.
#
# NOTE: The name of the variables used throughout the CMakeLists.txt files do
#       not necessairly compare to the name of the variables in the .px4board
#       file.
#
# ${CMAKE_CURRENT_SOURCE_DIR}/cmake/kconfig.cmake
include(kconfig)

# Output the platform and configuration being used 
# (PX4_PLATFORM is defined in kconfig and corresponds to the 
# CONFIG_BOARD_PLATFORM in boardconfig make file)
message(STATUS "PX4 config: ${PX4_CONFIG}")
message(STATUS "PX4 platform: ${PX4_PLATFORM}")

# Checks if the build is run from CLion and disables GCC poison macros to avoid 
# errors during CLion’s automatic compiler checks, which occur before NuttX 
# files are generated.
if($ENV{CLION_IDE})
	# CLion automatically executes some compiler commands after configuring the
	# project. This would fail on NuttX, as visibility.h tries to (indirectly)
	# include nuttx/config.h, which at that point does not exist yet
	add_definitions(-DPX4_DISABLE_GCC_POISON)
endif()

# If running posix platform, enable lockstep schedular. Lockstep Schedular is 
# used in simulation to allow synchronise the PX4 performance with simulation
# data.
if(${PX4_PLATFORM} STREQUAL "posix")
	if(ENABLE_LOCKSTEP_SCHEDULER)
		add_definitions(-DENABLE_LOCKSTEP_SCHEDULER)
		message(STATUS "PX4 lockstep: enabled")
	else()
		message(STATUS "PX4 lockstep: disabled")
	endif()
endif()

# Path for external PX4 modules configurable via cache variable
set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

# If external modules path is set, convert it to an absolute path
if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	get_filename_component(EXTERNAL_MODULES_LOCATION "${EXTERNAL_MODULES_LOCATION}" ABSOLUTE)
endif()

# Initialize global property to hold PX4 module config files
set_property(GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)

# Include platform-specific PX4 OS implementation
include(platforms/${PX4_PLATFORM}/cmake/px4_impl_os.cmake)

# Add platform-specific CMake directory to CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

# Include platform-specific init script if it exists
if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/init.cmake")
	include(init)
endif()

#=============================================================================
# PX4 PROJECT DEFINITION
#=============================================================================

# Create PX4 project
project(px4 CXX C ASM)

# Set the default CMake build type if not provided (based on PX4 platform)
if(NOT CMAKE_BUILD_TYPE)
    if(${PX4_PLATFORM} STREQUAL "nuttx")
        set(PX4_BUILD_TYPE "MinSizeRel")  # Default to MinSizeRel for NuttX
    else()
        set(PX4_BUILD_TYPE "RelWithDebInfo")  # Default to RelWithDebInfo otherwise
    endif()
    set(CMAKE_BUILD_TYPE ${PX4_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

# Choose max optimization level based on build type
if((CMAKE_BUILD_TYPE STREQUAL "Debug") OR (CMAKE_BUILD_TYPE STREQUAL "Coverage"))
    set(MAX_CUSTOM_OPT_LEVEL -O0)
elseif(CMAKE_BUILD_TYPE MATCHES "Sanitizer")
    set(MAX_CUSTOM_OPT_LEVEL -O1)
elseif(CMAKE_BUILD_TYPE MATCHES "Release")
    set(MAX_CUSTOM_OPT_LEVEL -O3)
else()
    if(px4_constrained_flash_build)
        set(MAX_CUSTOM_OPT_LEVEL -Os)
    else()
        set(MAX_CUSTOM_OPT_LEVEL -O2)
    endif()
endif()

# Populate the allowed CMake build types for cache editors
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

# If LTO (Link Time Optimization) is enabled, check for support and enable IPO
if(LTO)
    include(CheckIPOSupported)
    check_ipo_supported()
    message(AUTHOR_WARNING "LTO enabled: LTO is highly experimental and should not be used in production")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Set default package contact email
set(package-contact "px4users@googlegroups.com")

# Set C++ and C language standards and enforce them
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set binary output directories for all build types to PX4_BINARY_DIR
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PX4_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PX4_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PX4_BINARY_DIR})

#=============================================================================

# Attempt to use the gold linker on POSIX platforms if available (faster linking)
if(${PX4_PLATFORM} STREQUAL "posix")
	include(CMakeDependentOption)
	CMAKE_DEPENDENT_OPTION(USE_LD_GOLD
			"Use GNU gold linker" ON
			"NOT WIN32;NOT APPLE" OFF
	)

	if(USE_LD_GOLD)
		execute_process(COMMAND ${CMAKE_C_COMPILER} -fuse-ld=gold -Wl,--version ERROR_QUIET OUTPUT_VARIABLE LD_VERSION)
		if("${LD_VERSION}" MATCHES "GNU gold")
			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
			set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=gold")
		else()
			set(USE_LD_GOLD OFF)
		endif()
	endif()
endif()

#=============================================================================

# Set up install paths and related compiler options for POSIX builds
if(${PX4_PLATFORM} STREQUAL "posix")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # Required for loading code at runtime
    set(CMAKE_ENABLE_EXPORTS ON)             # Export symbols from px4 executable

    # Include code coverage tools if building with Coverage
    if(CMAKE_BUILD_TYPE MATCHES "Coverage")
        include(coverage)
    endif()

    include(sanitizers)           # Add sanitizer compilation options
    include(GNUInstallDirs)       # Use GNU standard installation directories

    if (NOT CMAKE_INSTALL_PREFIX)
        set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install path prefix" FORCE)
    endif()
endif()

# Enable ccache (if available) to speed up further builds
include(ccache)

#=============================================================================
# GET CHIP & CHIP MANUFACTURER
#=============================================================================

# Determine the chip being used for the model of microcontroller
px4_os_determine_build_chip()

# Confirm a valid chip manufacturer was found
if(NOT PX4_CHIP_MANUFACTURER)
	message(FATAL_ERROR "px4_os_determine_build_chip() needs to set PX4_CHIP_MANUFACTURER")
endif()

# Confirm a valid chip was found
if(NOT PX4_CHIP)
	message(FATAL_ERROR "px4_os_determine_build_chip() needs to set PX4_CHIP")
endif()

#=============================================================================
# Testing - Automatic unit and integration testing with CTest
# (Needs to be before setting the common compile flags)
#=============================================================================

# Optionally enable cmake testing (supported only on posix)
option(CMAKE_TESTING "Configure test targets" OFF)
if(${PX4_CONFIG} STREQUAL "px4_sitl_test")
	set(CMAKE_TESTING ON)
endif()
if(CMAKE_TESTING)
	include(CTest) # sets BUILD_TESTING variable
endif()

# enable test filtering to run only specific tests with the ctest -R regex functionality
set(TESTFILTER "" CACHE STRING "Filter string for ctest to selectively only run specific tests (ctest -R)")

# PX4 Google test integration
include(px4_add_gtest)
if(BUILD_TESTING)
	# Setting FUZZTEST_FUZZING_MODE=on enables ASAN, and is only supported with Clang
	if (("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang"))
		set(FUZZTEST_FUZZING_MODE ON)
	endif()
	add_subdirectory(test)
	fuzztest_setup_fuzzing_flags()
endif()

#=============================================================================
# BUILD FLAGS
#=============================================================================

# Include and apply common PX4 build flags
include(px4_add_common_flags)
px4_add_common_flags()
px4_os_add_flags()

#=============================================================================
# board cmake init (optional)
#=============================================================================

# Optionally include custom board initialization if cmake/init.cmake exists
if(EXISTS ${PX4_BOARD_DIR}/cmake/init.cmake)
	include(${PX4_BOARD_DIR}/cmake/init.cmake)
endif()

#=============================================================================
# message, and airframe generation
#=============================================================================

# Include PX4 metadata handling, generate message headers and airframe XML
include(px4_metadata)

# Add the msg subdirectory, excluding from global build targets
add_subdirectory(msg EXCLUDE_FROM_ALL)

# Generate airframes XML file for the chosen PX4 board
px4_generate_airframes_xml(BOARD ${PX4_BOARD})

#=============================================================================
# EXTERNAL PROJECTS
#=============================================================================

# Set the base directory for external build artifacts
set(ep_base ${PX4_BINARY_DIR}/external)
set_property(DIRECTORY PROPERTY EP_BASE ${ep_base})

# Create external project install directories; suppress CMake warnings
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base})
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install/lib)
link_directories(${ep_base}/Install/lib)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install/include)
include_directories(${ep_base}/Install/include)

#=============================================================================
# EXTERNAL MODULES
#=============================================================================

# Add external modules to the build if location is set
set(external_module_paths)
if (NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
    message(STATUS "External modules: ${EXTERNAL_MODULES_LOCATION}")
    add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src" external_modules)

    # Add each external module subdirectory and track its path
    foreach(external_module ${config_module_list_external})
        add_subdirectory(${EXTERNAL_MODULES_LOCATION}/src/${external_module} external_modules/${external_module})
        list(APPEND external_module_paths ${EXTERNAL_MODULES_LOCATION}/src/${external_module})
    endforeach()
endif()

#=============================================================================
# SUBDIRECTORIES
#=============================================================================

# Define interface libraries for parameters and events
add_library(parameters_interface INTERFACE)
add_library(kernel_parameters_interface INTERFACE)
add_library(events_interface INTERFACE)
add_library(kernel_events_interface INTERFACE)

# Include PX4 library-defining macros
include(px4_add_library)
# Add src/lib directory to build for generic libraries
add_subdirectory(src/lib EXCLUDE_FROM_ALL)

# Add platform-specific PX4 source directory
add_subdirectory(platforms/${PX4_PLATFORM}/src/px4)
# Add the platforms directory for platform-specific code
add_subdirectory(platforms EXCLUDE_FROM_ALL)

# Add board-specific directory if CMakeLists.txt exists
if(EXISTS "${PX4_BOARD_DIR}/CMakeLists.txt")
    add_subdirectory(${PX4_BOARD_DIR})
endif()

# Add main PX4 modules from config
foreach(module ${config_module_list})
    add_subdirectory(src/${module})
endforeach()

# Add events library after modules so it sees all source files
add_subdirectory(src/lib/events EXCLUDE_FROM_ALL)
# Metadata library needs PX4_MODULE_CONFIG_FILES; add after modules
add_subdirectory(src/lib/metadata EXCLUDE_FROM_ALL)

# Add parameters library last before firmware for dependency reasons
add_subdirectory(src/lib/parameters EXCLUDE_FROM_ALL)

# Link interface libraries based on platform and build configuration
if(${PX4_PLATFORM} STREQUAL "nuttx" AND NOT CONFIG_BUILD_FLAT)
    target_link_libraries(parameters_interface INTERFACE usr_parameters)
    target_link_libraries(kernel_parameters_interface INTERFACE parameters)
    target_link_libraries(events_interface INTERFACE usr_events)
    target_link_libraries(kernel_events_interface INTERFACE events)
else()
    target_link_libraries(parameters_interface INTERFACE parameters)
    target_link_libraries(events_interface INTERFACE events)
endif()

# Add the firmware subdirectory last to generate module built-ins
add_subdirectory(platforms/${PX4_PLATFORM})

#=============================================================================
# uORB graph generation: add a custom target 'uorb_graph'
#=============================================================================

# Set board-specific graph config and source paths for uORB graph
set(uorb_graph_config ${PX4_BOARD})
set(graph_module_list "")
foreach(module ${config_module_list})
    set(graph_module_list "${graph_module_list}" "--src-path" "src/${module}")
endforeach()

# Define custom command and target to generate uORB graph output
add_custom_command(OUTPUT ${uorb_graph_config}
    COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/uorb_graph/create.py
        ${graph_module_list} --src-path src/lib
        --merge-depends
        --exclude-path src/examples
        --exclude-path src/lib/parameters # FIXME: enable & fix
        --file ${PX4_SOURCE_DIR}/Tools/uorb_graph/graph_${uorb_graph_config}
    WORKING_DIRECTORY ${PX4_SOURCE_DIR}
    COMMENT "Generating uORB graph"
)
add_custom_target(uorb_graph DEPENDS ${uorb_graph_config})

# Include memory analysis and packaging scripts
include(bloaty)
include(metadata)
include(package)

# Custom target to install Python requirements with the configured python interpreter
add_custom_target(install_python_requirements
    COMMAND ${PYTHON_EXECUTABLE} -m pip install --break-system-packages --requirement ${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
    DEPENDS ${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
    USES_TERMINAL
)

# Optionally include final platform-specific build script
if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/finalize.cmake")
    include(finalize)
endif()