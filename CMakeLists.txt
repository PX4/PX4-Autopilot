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

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

cmake_policy(SET CMP0058 NEW)

set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "PX4 source directory" FORCE)
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE FILEPATH "PX4 binary directory" FORCE)

list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake)
include(px4_parse_function_args)

#=============================================================================
# git
#
include(px4_git)

execute_process(
	COMMAND git describe --exclude ext/* --always --tags
	OUTPUT_VARIABLE PX4_GIT_TAG
	OUTPUT_STRIP_TRAILING_WHITESPACE
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	)
message(STATUS "PX4 version: ${PX4_GIT_TAG}")

define_property(GLOBAL PROPERTY PX4_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 module libs"
                 FULL_DOCS "List of all PX4 module libraries"
                 )
define_property(GLOBAL PROPERTY PX4_MODULE_PATHS
                 BRIEF_DOCS "PX4 module paths"
                 FULL_DOCS "List of paths to all PX4 modules"
                 )
define_property(GLOBAL PROPERTY PX4_SRC_FILES
                 BRIEF_DOCS "src files from all PX4 modules & libs"
                 FULL_DOCS "SRC files from px4_add_{module,library}"
                 )
define_property(GLOBAL PROPERTY PX4_PUBLICATIONS
                 BRIEF_DOCS "PX4 publication topics"
                 FULL_DOCS "List of topics published by PX4 modules"
                 )
define_property(GLOBAL PROPERTY PX4_SUBSCRIPTIONS
                 BRIEF_DOCS "PX4 subscription topics"
                 FULL_DOCS "List of topics subscribed by PX4 modules"
                 )
#=============================================================================
# configuration
#

# default to px4_ros2_default if building within a ROS2 colcon environment
if((DEFINED ENV{COLCON_PREFIX_PATH}) AND (DEFINED ENV{ROS_VERSION}))
	if("$ENV{ROS_VERSION}" MATCHES "2")
		message(STATUS "ROS_VERSION: $ENV{ROS_VERSION}")
		set(CONFIG "px4_ros2_default" CACHE STRING "desired configuration") # TODO
	endif()
endif()

if(NOT CONFIG)
	set(CONFIG "px4_sitl_default" CACHE STRING "desired configuration")
endif()

include(px4_add_module)
set(config_module_list)

# Find Python
find_package(PythonInterp 3)
# We have a custom error message to tell users how to install python3.
if(NOT PYTHONINTERP_FOUND)
	message(FATAL_ERROR "Python 3 not found. Please install Python 3:\n"
		"    Ubuntu: sudo apt install python3 python3-dev python3-pip\n"
		"    macOS: brew install python")
endif()

option(PYTHON_COVERAGE "Python code coverage" OFF)
if(PYTHON_COVERAGE)
	message(STATUS "python coverage enabled")
	set(PYTHON_EXECUTABLE coverage run -p)
endif()

include(px4_config)
include(kconfig)
message(STATUS "PX4 config: ${PX4_CONFIG}")
message(STATUS "PX4 platform: ${PX4_PLATFORM}")


if(${PX4_PLATFORM} STREQUAL "posix")
	if(ENABLE_LOCKSTEP_SCHEDULER)
		add_definitions(-DENABLE_LOCKSTEP_SCHEDULER)
		message(STATUS "PX4 lockstep: enabled")
	else()
		message(STATUS "PX4 lockstep: disabled")
	endif()
endif()

# external modules
set(EXTERNAL_MODULES_LOCATION "" CACHE STRING "External modules source location")

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	get_filename_component(EXTERNAL_MODULES_LOCATION "${EXTERNAL_MODULES_LOCATION}" ABSOLUTE)
endif()

set_property(GLOBAL PROPERTY PX4_MODULE_CONFIG_FILES)

include(platforms/${PX4_PLATFORM}/cmake/px4_impl_os.cmake)
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)

# CMake build type (Debug Release RelWithDebInfo MinSizeRel Coverage)
if(NOT CMAKE_BUILD_TYPE)
	if(${PX4_PLATFORM} STREQUAL "nuttx")
		set(PX4_BUILD_TYPE "MinSizeRel")
	else()
		set(PX4_BUILD_TYPE "RelWithDebInfo")
	endif()

	set(CMAKE_BUILD_TYPE ${PX4_BUILD_TYPE} CACHE STRING "Build type" FORCE)
endif()

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

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage;AddressSanitizer;UndefinedBehaviorSanitizer")
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

#=============================================================================
# project definition
#
project(px4 CXX C ASM)

set(package-contact "px4users@googlegroups.com")

set(CMAKE_CXX_STANDARD 17)
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

if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/init.cmake")
	include(init)
endif()

#=============================================================================

# gold linker - use if available (posix only for now)
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

# Setup install paths
if(${PX4_PLATFORM} STREQUAL "posix")
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

include(ccache)

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

#=============================================================================
# get chip and chip manufacturer
#
px4_os_determine_build_chip()
if(NOT PX4_CHIP_MANUFACTURER)
	message(FATAL_ERROR "px4_os_determine_build_chip() needs to set PX4_CHIP_MANUFACTURER")
endif()
if(NOT PX4_CHIP)
	message(FATAL_ERROR "px4_os_determine_build_chip() needs to set PX4_CHIP")
endif()

#=============================================================================
# board cmake init (optional)
#
if(EXISTS ${PX4_BOARD_DIR}/cmake/init.cmake)
	include(${PX4_BOARD_DIR}/cmake/init.cmake)
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
# external projects
#
set(ep_base ${PX4_BINARY_DIR}/external)
set_property(DIRECTORY PROPERTY EP_BASE ${ep_base})

# add external project install folders to build
# add the directories so cmake won't warn
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base})
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install/lib)
link_directories(${ep_base}/Install/lib)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${ep_base}/Install/include)
include_directories(${ep_base}/Install/include)

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
if(${PX4_CONFIG} STREQUAL "px4_sitl_test")
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
endif()


#=============================================================================
# subdirectories
#
add_library(parameters_interface INTERFACE)

include(px4_add_library)
add_subdirectory(src/lib EXCLUDE_FROM_ALL)

add_subdirectory(platforms/${PX4_PLATFORM}/src/px4 EXCLUDE_FROM_ALL)

if(${PX4_PLATFORM} MATCHES "ros2") # TODO: fix
	add_subdirectory(platforms/common/work_queue EXCLUDE_FROM_ALL)
else()
	add_subdirectory(platforms EXCLUDE_FROM_ALL)
endif()

if(EXISTS "${PX4_BOARD_DIR}/CMakeLists.txt")
	add_subdirectory(${PX4_BOARD_DIR})
endif()

foreach(module ${config_module_list})
	add_subdirectory(src/${module})
endforeach()

# add events lib after modules and libs as it needs to know all source files (PX4_SRC_FILES)
add_subdirectory(src/lib/events EXCLUDE_FROM_ALL)
# metadata needs PX4_MODULE_CONFIG_FILES
add_subdirectory(src/lib/metadata EXCLUDE_FROM_ALL)

# must be the last module before firmware
add_subdirectory(src/lib/parameters EXCLUDE_FROM_ALL)
target_link_libraries(parameters_interface INTERFACE parameters)

# firmware added last to generate the builtin for included modules
add_subdirectory(platforms/${PX4_PLATFORM})


set(PX4_ORB_TOPIC_COUNT 0)

if(${PX4_PLATFORM} MATCHES "ros2") # TODO: fix

	configure_file(${CMAKE_SOURCE_DIR}/build/px4_fmu-v5x_default/uORBTopics.cpp ${CMAKE_BINARY_DIR}/uORBTopics.cpp COPYONLY)
	configure_file(${CMAKE_SOURCE_DIR}/build/px4_fmu-v5x_default/uORBTopics.hpp ${CMAKE_BINARY_DIR}/uORBTopics.hpp COPYONLY)

	set(PX4_ORB_TOPIC_COUNT 1)

else()

	get_property(publications GLOBAL PROPERTY PX4_PUBLICATIONS)
	get_property(subscriptions GLOBAL PROPERTY PX4_SUBSCRIPTIONS)

	# TODO: for now combine subsriptions and publications for complete topic list
	list(APPEND publications ${subscriptions})

	list(SORT publications)
	list(REMOVE_DUPLICATES publications)

	set(pub_all_topics)
	set(PX4_MSG_TYPE_ID)
	set(PX4_MSG_TOPIC_ID)
	set(PX4_MSG_TOPIC_ID_STRING)
	set(PX4_MSG_TOPIC_ORB_ID)

	set(PX4_ORB_DECLARE_STR)
	set(PX4_ORB_DEFINE_STR)
	set(PX4_ORB_HEADER_INCLUDE_STR)


	foreach(pub ${publications})
		#message(STATUS "pub: ${pub}")
		string(REPLACE " /" ";" pub ${pub})

		list(GET pub 0 pub_type)
		list(GET pub 1 pub_topic)

		string(REPLACE "::" ";" pub_type_split ${pub_type})
		list(GET pub_type_split 2 pub_type_simple_lower)

		# Pascal case to snake case (PubType -> pub_type)
		string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" pub_type_simple_lower "${pub_type_simple_lower}")
		string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" pub_type_simple_lower "${pub_type_simple_lower}")
		string(TOLOWER "${pub_type_simple_lower}" pub_type_simple_lower)

		#message(STATUS "pub: Type: ${pub_type}, Topic: ${pub_topic} (${pub_type_simple_lower})")




		# pub_type to include path (eg px4::msg::VehicleStatus => px4/msg/vehicle_status.hpp)
		#  temporary create px4/msg/vehicle_status.hpp which simply includes <uORB/topics/vehicle_status.h>




		list(APPEND pub_all_topics ${pub_topic})

		list(APPEND PX4_MSG_TYPE_ID ${pub_type})

		#list(APPEND PX4_MSG_TOPIC_ID ${pub_topic})
		set(PX4_MSG_TOPIC_ID "${PX4_MSG_TOPIC_ID}\t${pub_topic},\n")
		#set(PX4_MSG_TOPIC_ID_STRING "${PX4_MSG_TOPIC_ID_STRING}\t"case ORB_ID::${pub_topic}: return \"${pub_topic}\";"\n")
		set(PX4_MSG_TOPIC_ORB_ID "${PX4_MSG_TOPIC_ORB_ID}\tORB_ID(${pub_topic}),\n")

		list(APPEND PX4_MSG_TOPIC_ID_STRING
			"case ORB_ID::${pub_topic}: return \"${pub_topic}\";\n"
		)

		# PX4_MSG_TYPE_ID
		# PX4_MSG_TOPIC_ID

		# .h  ORB_DECLARE(actuator_controls_0);
		# .c  ORB_DEFINE(actuator_controls_0, struct actuator_controls_s, 48, __orb_actuator_controls_fields, static_cast<uint8_t>(ORB_ID::actuator_controls_0));


		set(PX4_ORB_DECLARE_STR
			"${PX4_ORB_DECLARE_STR}ORB_DECLARE(${pub_topic});\n")

		# ORB_DEFINE(actuator_armed, struct actuator_armed_s, 16, __orb_actuator_armed_fields, static_cast<uint8_t>(ORB_ID::actuator_armed));
		set(PX4_ORB_DEFINE_STR
			"${PX4_ORB_DEFINE_STR}ORB_DEFINE(${pub_topic}, ${pub_type}, px4_embedded::${pub_type_simple_lower}_s::SIZE_NO_PADDING, px4_embedded::${pub_type_simple_lower}_s::FIELDS, static_cast<uint8_t>(ORB_ID::${pub_topic}));\n")

		set(PX4_ORB_HEADER_INCLUDE_STR
			"${PX4_ORB_HEADER_INCLUDE_STR}#include <uORB/topics/${pub_type_simple_lower}.h>\n")

		math(EXPR PX4_ORB_TOPIC_COUNT "${PX4_ORB_TOPIC_COUNT}+1")

	endforeach()
	list(REMOVE_DUPLICATES PX4_MSG_TYPE_ID)
	#list(REMOVE_DUPLICATES PX4_MSG_TOPIC_ID)

	#message(STATUS "PX4_MSG_TYPE_ID: ${PX4_MSG_TYPE_ID}")
	#message(STATUS "PX4_MSG_TOPIC_ID: ${PX4_MSG_TOPIC_ID}")

	if(PX4_ORB_TOPIC_COUNT GREATER 0)
		configure_file(uORBTopics.cpp.in ${CMAKE_BINARY_DIR}/uORBTopics.cpp)
		configure_file(uORBTopics.hpp.in ${CMAKE_BINARY_DIR}/uORBTopics.hpp)
	endif()

	# .hpp enum ORB_ID
	# .cpp struct orb_metadat ORB_ID() array

	foreach(f ${msg_files})
		#message(STATUS "MSG: ${f}")
	endforeach()
endif()

if(PX4_ORB_TOPIC_COUNT GREATER 0)
	add_library(uorb_msgs ${uorb_headers} ${CMAKE_BINARY_DIR}/uORBTopics.cpp ${CMAKE_BINARY_DIR}/uORBTopics.hpp)
	add_dependencies(uorb_msgs prebuild_targets uorb_headers)
endif()

if(${PX4_PLATFORM} MATCHES "ros2") # TODO: fix
	ament_target_dependencies(uorb_msgs rclcpp std_msgs)
	rosidl_target_interfaces(uorb_msgs ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

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
		${graph_module_list} --src-path src/lib
		--merge-depends
		--exclude-path src/examples
		--exclude-path src/lib/parameters # FIXME: enable & fix
		--file ${PX4_SOURCE_DIR}/Tools/uorb_graph/graph_${uorb_graph_config}
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	COMMENT "Generating uORB graph"
)
add_custom_target(uorb_graph DEPENDS ${uorb_graph_config})


include(doxygen)
include(metadata)
include(package)

# install python requirements using configured python
add_custom_target(install_python_requirements
	COMMAND ${PYTHON_EXECUTABLE} -m pip install --requirement ${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
	DEPENDS ${PX4_SOURCE_DIR}/Tools/setup/requirements.txt
	USES_TERMINAL
)

if(EXISTS "${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/finalize.cmake")
	include(finalize)
endif()
