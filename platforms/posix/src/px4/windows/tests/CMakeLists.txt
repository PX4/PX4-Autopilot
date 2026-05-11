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

#
# Unit tests for the Windows POSIX shim layer. These targets only build
# when CMAKE_TESTING is on (BUILD_TESTING is set by include(CTest)).
# Tests are gated to Windows hosts because the shim headers are
# Windows-only by design; on non-Windows the .cpp files compile to a
# trivial sentinel test so the gtest runner still produces output.
#

if(NOT BUILD_TESTING)
	return()
endif()

# Shared static-init hook that silences the MSVC Debug CRT invalid
# parameter handler so tests that exercise bad-fd / bad-handle paths do
# not abort with a debug-time assertion dialog. Linked into every
# Windows shim test target as an EXTRA_SRC.
set(_px4_windows_shim_silencer
	${CMAKE_CURRENT_SOURCE_DIR}/test_main_silence.cpp
)

# Headers-only tests (libgen, dirent, sched, syslog, mman, time, unistd,
# getopt, fcntl). The translation unit pulls the shim headers via the
# existing windows_shim include path; the inline fcntl() routes
# O_NONBLOCK changes through ioctlsocket() so we need ws2_32 to satisfy
# the symbol even on the headers-only target.
px4_add_unit_gtest(
	SRC test_windows_shim_headers.cpp
	EXTRA_SRCS ${_px4_windows_shim_silencer}
	LINKLIBS ws2_32
)

# Source-defined shim tests (errno_map, env, sysconf, mman, flock, dlfcn,
# ids, if_query, resolver, socket loopback). Pulls in the standalone
# Windows shim sources directly so the tests do not need the full PX4
# platform/uORB stack.
set(_px4_windows_shim_root ${CMAKE_CURRENT_SOURCE_DIR}/..)

set(_px4_windows_shim_test_srcs
	${_px4_windows_shim_root}/posix/sys/errno_map.cpp
	${_px4_windows_shim_root}/posix/sys/sysconf.cpp
	${_px4_windows_shim_root}/posix/proc/env.cpp
	${_px4_windows_shim_root}/posix/proc/ids.cpp
	${_px4_windows_shim_root}/posix/fs/flock.cpp
	${_px4_windows_shim_root}/posix/fs/mman.cpp
	${_px4_windows_shim_root}/posix/lib/dlfcn.cpp
	${_px4_windows_shim_root}/posix/net/if_query.cpp
	${_px4_windows_shim_root}/posix/net/resolver.cpp
)

px4_add_unit_gtest(
	SRC test_windows_shim_runtime.cpp
	EXTRA_SRCS ${_px4_windows_shim_test_srcs} ${_px4_windows_shim_silencer}
	INCLUDES
		${PX4_SOURCE_DIR}/platforms/posix/include/windows_shim
		${_px4_windows_shim_root}/include
	LINKLIBS ws2_32 iphlpapi
)

# poll() inline implementation - header-only test.
px4_add_unit_gtest(
	SRC test_windows_shim_poll.cpp
	EXTRA_SRCS ${_px4_windows_shim_silencer}
	INCLUDES
		${PX4_SOURCE_DIR}/platforms/posix/include/windows_shim
	LINKLIBS ws2_32
)

# I/O helpers (pipe, fsync, dprintf, mkdir, lstat). Header-only; the
# POSIX names route to MSVC CRT primitives via inline shim wrappers.
px4_add_unit_gtest(
	SRC test_windows_shim_io.cpp
	EXTRA_SRCS ${_px4_windows_shim_silencer}
	INCLUDES
		${PX4_SOURCE_DIR}/platforms/posix/include/windows_shim
)
