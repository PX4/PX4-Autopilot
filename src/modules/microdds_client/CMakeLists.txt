############################################################################
#
# Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

px4_add_git_submodule(TARGET git_micro_xrce_dds_client PATH "${CMAKE_CURRENT_LIST_DIR}/Micro-XRCE-DDS-Client")

# If a toolchain file is given, explicitly define its path when building the Micro-XRCE-DDS-CLient
if(CMAKE_TOOLCHAIN_FILE MATCHES "cmake$")
	set(microxrceddsclient_toolchain ${CMAKE_TOOLCHAIN_FILE})
elseif(CMAKE_TOOLCHAIN_FILE)
	set(microxrceddsclient_toolchain ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake/${CMAKE_TOOLCHAIN_FILE}.cmake)
endif()

# Include NuttX headers
get_property(include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
set(c_flags_with_includes "${CMAKE_C_FLAGS}")
set(cxx_flags_with_includes "${CMAKE_CXX_FLAGS}")
foreach(dir ${include_dirs})
	set(c_flags_with_includes "${c_flags_with_includes} -I${dir}")
	set(cxx_flags_with_includes "${cxx_flags_with_includes} -I${dir}")
endforeach()

set(lib_dir ${CMAKE_INSTALL_LIBDIR})
if("${lib_dir}" STREQUAL "")
	set(lib_dir "lib")
endif()

set(microxrceddsclient_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/Micro-XRCE-DDS-Client)
set(microxrceddsclient_build_dir ${CMAKE_CURRENT_BINARY_DIR})

include(ExternalProject)

ExternalProject_Add(
	libmicroxrceddsclient_project
	PREFIX      ${microxrceddsclient_build_dir}
	SOURCE_DIR  ${microxrceddsclient_src_dir}
	INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
	CMAKE_CACHE_ARGS
		-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
		-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
		-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
		-DCMAKE_C_FLAGS:STRING=${c_flags_with_includes}
		-DCMAKE_CXX_FLAGS:STRING=${cxx_flags_with_includes}
		-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
		-DUCLIENT_PIC:BOOL=OFF
		-DUCLIENT_PROFILE_TCP:BOOL=OFF
		-DUCLIENT_PROFILE_UDP:BOOL=ON
		-DUCLIENT_PROFILE_SERIAL:BOOL=ON
		-DUCLIENT_PROFILE_DISCOVERY:BOOL=OFF
		-DUCLIENT_PROFILE_CUSTOM_TRANSPORT:BOOL=ON
		-DUCLIENT_PLATFORM_POSIX:BOOL=ON
		-DUCLIENT_BUILD_MICROCDR:BOOL=ON
		-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}
		-DCMAKE_PREFIX_PATH:PATH=${CMAKE_CURRENT_BINARY_DIR}
		-DCMAKE_TOOLCHAIN_FILE:STRING=${microxrceddsclient_toolchain}
	BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${lib_dir}/libmicroxrcedds_client.a
	BUILD_BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${lib_dir}/libmicrocdr.a
	DEPENDS prebuild_targets git_micro_xrce_dds_client
    )

add_library(libmicroxrceddsclient STATIC IMPORTED GLOBAL)
set_target_properties(libmicroxrceddsclient
	PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${lib_dir}/libmicroxrcedds_client.a
)

add_library(libmicrocdr STATIC IMPORTED GLOBAL)
set_target_properties(libmicrocdr
	PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${lib_dir}/libmicrocdr.a
)

add_library(microxrceddsclient INTERFACE)
add_dependencies(microxrceddsclient libmicroxrceddsclient)
add_dependencies(microxrceddsclient libmicrocdr)
add_dependencies(microxrceddsclient libmicroxrceddsclient_project)
target_include_directories(microxrceddsclient INTERFACE ${microxrceddsclient_build_dir}/include)

px4_add_module(
	MODULE modules__microdds_client
	MAIN microdds_client
	STACK_MAIN 8000
	SRCS
		microdds_client.cpp
	DEPENDS
		microxrceddsclient
		libmicroxrceddsclient
		libmicrocdr
		uorb_ucdr_headers
	)

add_dependencies(modules__microdds_client topic_bridge_files)
