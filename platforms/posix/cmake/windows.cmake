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

# Windows support here is a Win32 backend for the POSIX platform, not a
# separate PX4 platform. Keep the backend source list and Win32 link policy
# out of platforms/posix/CMakeLists.txt so the top-level POSIX build remains
# about platform selection rather than Win32 implementation detail.

set(PX4_POSIX_WINDOWS_ROOT "${PX4_SOURCE_DIR}/platforms/posix/src/px4/windows")
set(PX4_POSIX_WINDOWS_PRIVATE_INCLUDE_DIR "${PX4_POSIX_WINDOWS_ROOT}/include")
set(PX4_POSIX_WINDOWS_DYNAMIC_MODULE_HOST_EXPORTS
	px4_log_modulename
)

function(px4_posix_windows_append_sources out_var)
	list(APPEND ${out_var}
		${PX4_POSIX_WINDOWS_ROOT}/runtime/init.cpp

		${PX4_POSIX_WINDOWS_ROOT}/posix/sys/errno_map.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/sys/ioctl.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/sys/sysconf.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/sys/termios.cpp

		${PX4_POSIX_WINDOWS_ROOT}/posix/proc/env.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/proc/ids.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/proc/sched.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/proc/user.cpp

		${PX4_POSIX_WINDOWS_ROOT}/posix/fs/flock.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/fs/mman.cpp

		${PX4_POSIX_WINDOWS_ROOT}/posix/net/if_query.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/net/resolver.cpp
		${PX4_POSIX_WINDOWS_ROOT}/posix/net/socket.cpp

		${PX4_POSIX_WINDOWS_ROOT}/posix/lib/dlfcn.cpp

		${PX4_POSIX_WINDOWS_ROOT}/shell/shell.cpp
		${PX4_POSIX_WINDOWS_ROOT}/shell/embedded_backend_stub.cpp
	)
	if(MSVC)
		list(APPEND ${out_var}
			${PX4_POSIX_WINDOWS_ROOT}/posix/proc/pthread.cpp
			${PX4_POSIX_WINDOWS_ROOT}/posix/sys/time.cpp
			${PX4_POSIX_WINDOWS_ROOT}/posix/net/socket_msvc.cpp
		)
	else()
		list(APPEND ${out_var}
			${PX4_POSIX_WINDOWS_ROOT}/posix/net/socket_wrap.cpp
		)
	endif()
	set(${out_var} "${${out_var}}" PARENT_SCOPE)
endfunction()

function(px4_posix_windows_configure_link_templates)
	if(MSVC)
		return()
	endif()

	# PX4 links many static archives with cross-archive symbol cycles
	# (uORB <-> platform <-> modules, winsock referenced from both daemon
	# server and uxrce_dds_client). GNU ld's default single-pass static
	# archive resolution can leave unresolved WinSock import symbols.
	# Wrap <LINK_LIBRARIES> in a group so ld rescans until convergence.
	string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> -Wl,--end-group"
		CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE}")
	string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> -Wl,--end-group"
		CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_LINK_EXECUTABLE}")

	set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE}" PARENT_SCOPE)
	set(CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_LINK_EXECUTABLE}" PARENT_SCOPE)
endfunction()

function(px4_posix_windows_configure_target target_name)
	target_include_directories(${target_name}
		PRIVATE
			${PX4_POSIX_WINDOWS_PRIVATE_INCLUDE_DIR}
	)

	set_target_properties(${target_name} PROPERTIES ENABLE_EXPORTS ON)

	if(MSVC)
		set_target_properties(${target_name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
		target_compile_options(${target_name} PRIVATE /bigobj)

		# Reproducible builds: zero out the PE COFF TimeDateStamp and the debug
		# directory timestamp so byte-identical sources produce a byte-identical
		# px4.exe. Without /Brepro every link embeds time(NULL), making CI
		# artifact diffing and supply-chain hashing meaningless.
		target_compile_options(${target_name} PRIVATE /Brepro)
		target_link_options(${target_name} PRIVATE /Brepro)

		# WINDOWS_EXPORT_ALL_SYMBOLS only scans objects owned directly by
		# px4.exe. Dynamic .px4mod modules also need host APIs that come from
		# PX4 static libraries, so export those entry points explicitly.
		foreach(host_export IN LISTS PX4_POSIX_WINDOWS_DYNAMIC_MODULE_HOST_EXPORTS)
			target_link_options(${target_name} PRIVATE "/EXPORT:${host_export}")
		endforeach()
	else()
		# Force an import library (`libpx4.dll.a`) to be generated alongside
		# px4.exe so DYNAMIC `.px4mod` modules can link against it.
		target_link_options(${target_name} PRIVATE -Wl,--export-all-symbols)

		# Route bare WinSock calls through socket_wrap.cpp so WSAGetLastError()
		# is translated into errno before PX4 logs strerror(errno).
		target_link_options(${target_name} PRIVATE
			-Wl,--wrap=socket
			-Wl,--wrap=bind
			-Wl,--wrap=listen
			-Wl,--wrap=accept
			-Wl,--wrap=connect
			-Wl,--wrap=setsockopt
			-Wl,--wrap=shutdown
			-Wl,--wrap=recv
			-Wl,--wrap=send
			-Wl,--wrap=recvfrom
			-Wl,--wrap=sendto
			-Wl,--wrap=strerror
		)
	endif()

	# Make ctest run `wine px4.exe ...` transparently.
	find_program(_px4_wine NAMES wine wine64)
	if(_px4_wine)
		set_target_properties(${target_name} PROPERTIES CROSSCOMPILING_EMULATOR "${_px4_wine}")
	endif()
endfunction()

function(px4_posix_windows_link_libraries target_name)
	# Keep Win32 import libs at the end of the link line. This is ordering
	# only: every PX4 static archive has already been scanned before these
	# are used to resolve platform symbols.
	target_link_libraries(${target_name}
		PRIVATE
			ws2_32
			iphlpapi
			dbghelp
			psapi
			winmm    # timeBeginPeriod / timeEndPeriod for 1 ms timer resolution
	)

	if(NOT MSVC)
		target_link_libraries(${target_name} PRIVATE winpthread)
	endif()
endfunction()

# Wire up the shim unit tests under platforms/posix/src/px4/windows/tests
# when CMake testing is configured. The tests are gated to Windows hosts.
function(px4_posix_windows_add_tests)
	if(NOT BUILD_TESTING)
		return()
	endif()
	add_subdirectory(${PX4_POSIX_WINDOWS_ROOT}/tests
		${CMAKE_BINARY_DIR}/platforms/posix/src/px4/windows/tests)
endfunction()
