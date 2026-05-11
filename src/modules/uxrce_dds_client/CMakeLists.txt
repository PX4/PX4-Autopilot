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

if(${CMAKE_VERSION} VERSION_LESS_EQUAL "3.15")
	message(WARNING "skipping uxrce_dds_client, Micro-XRCE-DDS-Client needs to be fixed to work with CMAKE_VERSION ${CMAKE_VERSION}")

else()

	set(microxrceddsclient_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/Micro-XRCE-DDS-Client)
	set(microxrceddsclient_build_dir ${CMAKE_CURRENT_BINARY_DIR})

	px4_add_git_submodule(TARGET git_micro_xrce_dds_client PATH "${microxrceddsclient_src_dir}")

	# If a toolchain file is given, explicitly define its path when building the Micro-XRCE-DDS-CLient
	if(CMAKE_TOOLCHAIN_FILE MATCHES "cmake$")
		# CMake accepts relative toolchain paths on the command line (resolved
		# against the source dir) but ExternalProject runs in its own working
		# directory, so we have to hand down an absolute path.
		if(NOT IS_ABSOLUTE "${CMAKE_TOOLCHAIN_FILE}")
			get_filename_component(microxrceddsclient_toolchain
				"${CMAKE_TOOLCHAIN_FILE}" ABSOLUTE BASE_DIR "${PX4_SOURCE_DIR}")
		else()
			set(microxrceddsclient_toolchain ${CMAKE_TOOLCHAIN_FILE})
		endif()
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

	set(microxrceddsclient_make_program ${CMAKE_MAKE_PROGRAM})
	if(microxrceddsclient_make_program)
		file(TO_CMAKE_PATH "${microxrceddsclient_make_program}" microxrceddsclient_make_program)
	endif()

	# On native Windows, force the libmicroxrceddsclient sub-build (and the
	# nested microcdr/uclient ExternalProjects it spawns via SuperBuild.cmake)
	# to use a Make-based generator instead of inheriting the parent's Ninja
	# generator. The Micro-XRCE-DDS-Client SuperBuild nests ExternalProjects
	# three deep (libmicroxrceddsclient_project -> microcdr -> uclient); when
	# all three layers run Ninja concurrently, the inner cmake's
	# `ninja -t recompact` and `ninja -t restat build.ninja` steps race the
	# outer ninja that still holds microcdr-build/build.ninja open, producing
	# `ninja: error: failed recompaction: Permission denied` deterministically
	# on Windows where files cannot be replaced while another process has a
	# handle on them. Switching the sub-build to NMake (MSVC) or MinGW
	# Makefiles (MinGW) side-steps the lock collision entirely. The
	# microcdr/uclient compile is small enough that the loss of Ninja's
	# parallelism is not measurable in the overall build.
	#
	# Prefer nmake when it is on PATH (MSVC / vcvars64 shell), fall back to
	# mingw32-make (msys2 mingw-w64, on PATH after prepending
	# C:/msys64/mingw64/bin), and warn only if neither is available.
	set(microxrceddsclient_extra_args)
	if(WIN32)
		find_program(microxrceddsclient_nmake_program nmake)
		find_program(microxrceddsclient_mingw_make_program mingw32-make)
		if(microxrceddsclient_nmake_program)
			file(TO_CMAKE_PATH "${microxrceddsclient_nmake_program}"
				microxrceddsclient_make_program)
			list(APPEND microxrceddsclient_extra_args
				CMAKE_GENERATOR "NMake Makefiles")
		elseif(microxrceddsclient_mingw_make_program)
			file(TO_CMAKE_PATH "${microxrceddsclient_mingw_make_program}"
				microxrceddsclient_make_program)
			list(APPEND microxrceddsclient_extra_args
				CMAKE_GENERATOR "MinGW Makefiles")
		else()
			# Without a Make-based generator we can't switch; fall through
			# to the parent's Ninja generator and warn so the developer
			# sees why the inner build may flake on `failed recompaction`.
			message(WARNING
				"uxrce_dds_client: neither nmake nor mingw32-make found on "
				"PATH; the libmicroxrceddsclient sub-build will inherit the "
				"parent Ninja generator and may hit "
				"'ninja: error: failed recompaction: Permission denied'. "
				"Run from a Visual Studio Developer Command Prompt "
				"(vcvars64) for nmake.exe, or prepend C:/msys64/mingw64/bin "
				"to PATH for mingw32-make.exe.")
		endif()
	endif()

	set(microxrceddsclient_library ${CMAKE_CURRENT_BINARY_DIR}/${lib_dir}/${CMAKE_STATIC_LIBRARY_PREFIX}microxrcedds_client${CMAKE_STATIC_LIBRARY_SUFFIX})
	set(microcdr_library ${CMAKE_CURRENT_BINARY_DIR}/${lib_dir}/${CMAKE_STATIC_LIBRARY_PREFIX}microcdr${CMAKE_STATIC_LIBRARY_SUFFIX})

	include(ExternalProject)
	ExternalProject_Add(
		libmicroxrceddsclient_project
		PREFIX      ${microxrceddsclient_build_dir}
		SOURCE_DIR  ${microxrceddsclient_src_dir}
		INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
		${microxrceddsclient_extra_args}
		CMAKE_CACHE_ARGS
			-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}
			-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}
			-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
			-DCMAKE_C_FLAGS:STRING=${c_flags_with_includes}
			-DCMAKE_CXX_FLAGS:STRING=${cxx_flags_with_includes}
			-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
			# Forward the generator + make program so nested superbuilds
			# (microcdr, uclient) can configure themselves without a second
			# round of auto-detection.
			-DCMAKE_MAKE_PROGRAM:FILEPATH=${microxrceddsclient_make_program}
			-DUCLIENT_PIC:BOOL=OFF
			-DUCLIENT_PROFILE_TCP:BOOL=OFF
			-DUCLIENT_PROFILE_UDP:BOOL=ON
			-DUCLIENT_PROFILE_SERIAL:BOOL=ON
			-DUCLIENT_PROFILE_DISCOVERY:BOOL=OFF
			-DUCLIENT_PROFILE_CUSTOM_TRANSPORT:BOOL=OFF
			-DUCLIENT_PROFILE_MULTITHREAD:BOOL=OFF
			-DUCLIENT_PROFILE_SHARED_MEMORY:BOOL=OFF
			# Pick the right transport platform: on Windows let the uclient
			# CMakeLists auto-detect (UCLIENT_PLATFORM_WINDOWS) rather than
			# forcing POSIX — its POSIX transport pulls in <poll.h>.
			-DUCLIENT_PLATFORM_POSIX:BOOL=$<IF:$<OR:$<BOOL:${WIN32}>,$<BOOL:${MINGW}>>,OFF,ON>
			-DUCLIENT_BUILD_MICROCDR:BOOL=ON
			-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}
			-DCMAKE_PREFIX_PATH:PATH=${CMAKE_CURRENT_BINARY_DIR}
			-DCMAKE_TOOLCHAIN_FILE:STRING=${microxrceddsclient_toolchain}
		BUILD_BYPRODUCTS
			${microxrceddsclient_library}
			${microcdr_library}
		DEPENDS prebuild_targets git_micro_xrce_dds_client
		LOG_CONFIGURE true
		LOG_BUILD true
		LOG_INSTALL true
		LOG_OUTPUT_ON_FAILURE true
	)

	add_library(libmicroxrceddsclient STATIC IMPORTED GLOBAL)
	set_target_properties(libmicroxrceddsclient
		PROPERTIES IMPORTED_LOCATION ${microxrceddsclient_library}
	)

	# udp_transport_windows.c references getaddrinfo/freeaddrinfo/socket
	# from ws2_32 and adapter helpers from iphlpapi. CMake places this
	# IMPORTED archive at the very end of the px4 link line, after the
	# platform `target_link_libraries(px4 ws2_32 …)` call, so we have to
	# attach the import libs to the archive itself for GNU ld to resolve
	# the symbols on the single forward pass.
	if(WIN32 OR MINGW)
		set_property(TARGET libmicroxrceddsclient
			APPEND PROPERTY INTERFACE_LINK_LIBRARIES ws2_32 iphlpapi)
	endif()

	add_library(libmicrocdr STATIC IMPORTED GLOBAL)
	set_target_properties(libmicrocdr
		PROPERTIES IMPORTED_LOCATION ${microcdr_library}
	)

	add_library(microxrceddsclient INTERFACE)
	add_dependencies(microxrceddsclient libmicroxrceddsclient)
	add_dependencies(microxrceddsclient libmicrocdr)
	add_dependencies(microxrceddsclient libmicroxrceddsclient_project)
	target_include_directories(microxrceddsclient INTERFACE ${microxrceddsclient_build_dir}/include)


	add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dds_topics.h
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_dds_topics.py
			--client-outdir ${CMAKE_CURRENT_BINARY_DIR}
			--dds-topics-file ${CMAKE_CURRENT_SOURCE_DIR}/dds_topics.yaml
			--template_file ${CMAKE_CURRENT_SOURCE_DIR}/dds_topics.h.em
		DEPENDS
			${CMAKE_CURRENT_SOURCE_DIR}/generate_dds_topics.py
			${CMAKE_CURRENT_SOURCE_DIR}/dds_topics.yaml
			${CMAKE_CURRENT_SOURCE_DIR}/dds_topics.h.em
		COMMENT "Generating XRCE-DDS topic bridge"
	)
	add_custom_target(topic_bridge_files DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/dds_topics.h)

	px4_add_module(
		MODULE modules__uxrce_dds_client
		MAIN uxrce_dds_client
		STACK_MAIN 9000
		INCLUDES
			${CMAKE_CURRENT_SOURCE_DIR}
		COMPILE_FLAGS
			#-O0
			# -DDEBUG_BUILD
			${MAX_CUSTOM_OPT_LEVEL}
		SRCS
			uxrce_dds_client.cpp
			uxrce_dds_client.h
			vehicle_command_srv.cpp
			vehicle_command_srv.h
			srv_base.cpp
			srv_base.h
		DEPENDS
			git_micro_xrce_dds_client
			libmicroxrceddsclient_project
			microxrceddsclient
			libmicroxrceddsclient
			libmicrocdr
			timesync
			topic_bridge_files
			uorb_ucdr_headers
		MODULE_CONFIG
			module.yaml
		)
endif()
