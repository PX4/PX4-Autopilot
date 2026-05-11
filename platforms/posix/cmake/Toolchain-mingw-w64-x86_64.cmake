############################################################################
#
# PX4 MinGW-w64 x86_64 cross toolchain.
#
# Used to cross-compile px4_sitl_default from a POSIX host to a native
# Windows x86_64 executable via the MinGW-w64 GCC port. Pick the
# *-posix* variant (winpthreads) so std::thread, pthread, and
# std::mutex work.
#
# Usage (from PX4-Autopilot repo root):
#   CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=Toolchain-mingw-w64-x86_64" \
#       make px4_sitl_default
#
# The bare toolchain name is resolved through CMAKE_MODULE_PATH, which
# cmake/kconfig.cmake extends with platforms/${PX4_PLATFORM}/cmake.
############################################################################

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(TOOLCHAIN_PREFIX x86_64-w64-mingw32)

set(MINGW_SEARCH_PATHS)

if(CMAKE_HOST_WIN32)
	list(APPEND MINGW_SEARCH_PATHS
		"$ENV{MINGW_PREFIX}/bin"
		"C:/msys64/mingw64/bin")
endif()

find_program(MINGW_REAL_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}-gcc-posix ${TOOLCHAIN_PREFIX}-gcc HINTS ${MINGW_SEARCH_PATHS})
find_program(MINGW_REAL_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}-g++-posix ${TOOLCHAIN_PREFIX}-g++ HINTS ${MINGW_SEARCH_PATHS})
find_program(MINGW_REAL_RC_COMPILER NAMES ${TOOLCHAIN_PREFIX}-windres HINTS ${MINGW_SEARCH_PATHS})
find_program(MINGW_AR NAMES ${TOOLCHAIN_PREFIX}-ar HINTS ${MINGW_SEARCH_PATHS})
find_program(MINGW_RANLIB NAMES ${TOOLCHAIN_PREFIX}-ranlib HINTS ${MINGW_SEARCH_PATHS})

if(NOT MINGW_REAL_C_COMPILER OR NOT MINGW_REAL_CXX_COMPILER)
	if(CMAKE_HOST_WIN32)
		message(FATAL_ERROR
			"MinGW-w64 (${TOOLCHAIN_PREFIX}) not found. "
			"Install the MSYS2 mingw-w64-x86_64 toolchain and add the MinGW bin directory "
			"(for example C:/msys64/mingw64/bin) to PATH.")
	else()
		message(FATAL_ERROR
			"MinGW-w64 (${TOOLCHAIN_PREFIX}) not found. "
			"Install with: apt-get install mingw-w64 g++-mingw-w64-x86-64-posix gcc-mingw-w64-x86-64-posix")
	endif()
endif()

get_filename_component(MINGW_BIN_DIR "${MINGW_REAL_C_COMPILER}" DIRECTORY)

if(CMAKE_HOST_WIN32)
	# The GCC driver is enough to locate cc1/ld, but child tools still need
	# the MinGW DLLs from the same bin directory.
	set(ENV{PATH} "${MINGW_BIN_DIR};$ENV{PATH}")

	set(MINGW_WRAPPER_DIR "${CMAKE_BINARY_DIR}/mingw_toolchain")
	set(MINGW_WRAPPER_PATH "${MINGW_BIN_DIR};%SystemRoot%/system32;%SystemRoot%;%SystemRoot%/System32/Wbem")
	file(MAKE_DIRECTORY "${MINGW_WRAPPER_DIR}")
	file(WRITE "${MINGW_WRAPPER_DIR}/gcc.cmd"
		"@echo off\r\n"
		"set \"PATH=${MINGW_WRAPPER_PATH}\"\r\n"
		"\"${MINGW_REAL_C_COMPILER}\" %*\r\n"
		"exit /b %ERRORLEVEL%\r\n")
	file(WRITE "${MINGW_WRAPPER_DIR}/gxx.cmd"
		"@echo off\r\n"
		"set \"PATH=${MINGW_WRAPPER_PATH}\"\r\n"
		"\"${MINGW_REAL_CXX_COMPILER}\" %*\r\n"
		"exit /b %ERRORLEVEL%\r\n")
	file(WRITE "${MINGW_WRAPPER_DIR}/windres.cmd"
		"@echo off\r\n"
		"set \"PATH=${MINGW_WRAPPER_PATH}\"\r\n"
		"\"${MINGW_REAL_RC_COMPILER}\" %*\r\n"
		"exit /b %ERRORLEVEL%\r\n")

	set(MINGW_C_COMPILER "${MINGW_WRAPPER_DIR}/gcc.cmd")
	set(MINGW_CXX_COMPILER "${MINGW_WRAPPER_DIR}/gxx.cmd")
	set(MINGW_RC_COMPILER "${MINGW_WRAPPER_DIR}/windres.cmd")
else()
	set(MINGW_C_COMPILER "${MINGW_REAL_C_COMPILER}")
	set(MINGW_CXX_COMPILER "${MINGW_REAL_CXX_COMPILER}")
	set(MINGW_RC_COMPILER "${MINGW_REAL_RC_COMPILER}")
endif()

set(CMAKE_C_COMPILER   ${MINGW_C_COMPILER})
set(CMAKE_CXX_COMPILER ${MINGW_CXX_COMPILER})
set(CMAKE_RC_COMPILER  ${MINGW_RC_COMPILER})
set(CMAKE_AR           ${MINGW_AR})
set(CMAKE_RANLIB       ${MINGW_RANLIB})

# Find headers/libs only in the target sysroot, but let us run host tools
# (python, etc.) during the build (generators, kconfig, msg-gen).
# PACKAGE uses BOTH so find_package() can locate packages installed by
# nested ExternalProject builds (microcdr under the PX4 build tree) as
# well as packages living in the MinGW sysroot.
if(CMAKE_HOST_WIN32)
	get_filename_component(MINGW_PREFIX "${MINGW_BIN_DIR}" DIRECTORY)
	set(CMAKE_FIND_ROOT_PATH
		"${MINGW_PREFIX}"
		"${MINGW_PREFIX}/${TOOLCHAIN_PREFIX}")
else()
	set(CMAKE_FIND_ROOT_PATH /usr/${TOOLCHAIN_PREFIX})
endif()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# MinGW lacks many POSIX headers. Put our shim directory in the search
# path ahead of the MinGW sysroot — do it here (rather than in the posix
# platform layer only) so that nested ExternalProject builds
# (Micro-XRCE-DDS-Client) using this toolchain also resolve <poll.h>,
# <sys/statfs.h>, <netdb.h>, <net/if.h>, <time.h> _r wrappers, etc.
get_filename_component(_px4_windows_shim_dir
	"${CMAKE_CURRENT_LIST_DIR}/../include/windows_shim" ABSOLUTE)
include_directories(BEFORE SYSTEM "${_px4_windows_shim_dir}")

# Vendored upstream code (Micro-XRCE-DDS-Client, CycloneDDS, libvnc) writes
# `#include <Windows.h>`. MinGW only ships the lowercase header, so on
# case-sensitive hosts (Linux, macOS) we need an uppercase alias. On
# case-insensitive hosts (Windows itself) the alias must NOT exist on the
# include path: a shim `Windows.h` would match `#include <windows.h>` first
# and `#pragma once`-recurse to itself, hiding the real header. Generate
# the alias under the build tree only when we actually need it.
if(NOT CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
	set(_px4_windows_case_alias_dir ${CMAKE_BINARY_DIR}/windows_case_alias)
	file(WRITE ${_px4_windows_case_alias_dir}/Windows.h
		"#pragma once\n#include <windows.h>\n")
	include_directories(BEFORE SYSTEM ${_px4_windows_case_alias_dir})
endif()

# Target Windows 10 (1803+) so AF_UNIX is available in WinSock2.
add_compile_definitions(
	_WIN32_WINNT=0x0A00       # Windows 10
	WINVER=0x0A00
	__PX4_WINDOWS             # PX4 platform selector
	NOMINMAX                  # avoid <windows.h> min/max macros
	WIN32_LEAN_AND_MEAN
	_USE_MATH_DEFINES
	__USE_MINGW_ANSI_STDIO=1  # make printf accept PRIu64 / %llu / %zu
)

# PX4 uses `#pragma pack(push, 1)` around structs with multi-bit bitfields
# (e.g. sixteen 11-bit RC channel fields in src/lib/rc/crsf.cpp). MinGW
# defaults to the MSVC bitfield ABI, which lays those out differently
# from Linux GCC (each storage unit padded to the declared type's
# alignment instead of packed bit-adjacent). Force the Itanium/SysV
# layout so decoded wire formats match across platforms.
add_compile_options(-mno-ms-bitfields)

# Produce a statically linked .exe so the user does not need to ship
# libgcc_s, libstdc++, and winpthread DLLs separately.
set(CMAKE_EXE_LINKER_FLAGS_INIT    "-static -static-libgcc -static-libstdc++")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-static-libgcc -static-libstdc++")

# ctest runs the PE binary via wine through the CROSSCOMPILING_EMULATOR
# property set on the px4 target (platforms/posix/CMakeLists.txt),
# rather than CMAKE_CROSSCOMPILING_EMULATOR here, because the latter
# does not reliably propagate across CMake reconfigures.
