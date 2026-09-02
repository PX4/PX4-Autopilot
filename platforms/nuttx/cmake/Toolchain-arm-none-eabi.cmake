# arm-none-eabi-gcc toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(triple arm-none-eabi)
set(CMAKE_LIBRARY_ARCHITECTURE ${triple})
set(TOOLCHAIN_PREFIX ${triple})

# The supported arm-none-eabi-gcc version. PX4 NuttX firmware is built and tested
# with exactly this compiler so release binaries are reproducible. Bump this together
# with the CI toolchain. Override with PX4_ARM_TOOLCHAIN_VERSION.
set(PX4_ARM_GCC_VERSION_REQUIRED "13.2.1")

if(NOT PX4_ARM_TOOLCHAIN_VERSION AND DEFINED ENV{PX4_ARM_TOOLCHAIN_VERSION})
	set(PX4_ARM_TOOLCHAIN_VERSION "$ENV{PX4_ARM_TOOLCHAIN_VERSION}")
endif()
if(NOT PX4_ARM_TOOLCHAIN_VERSION)
	set(PX4_ARM_TOOLCHAIN_VERSION "${PX4_ARM_GCC_VERSION_REQUIRED}")
endif()
string(TOLOWER "${PX4_ARM_TOOLCHAIN_VERSION}" PX4_ARM_TOOLCHAIN_VERSION_LOWER)
if(PX4_ARM_TOOLCHAIN_VERSION_LOWER MATCHES "^(any|none|off|0|false)$")
	set(PX4_ARM_TOOLCHAIN_ENFORCE FALSE)
else()
	set(PX4_ARM_TOOLCHAIN_ENFORCE TRUE)
endif()

# Locate the toolchain:
#   1. PX4_ARM_TOOLCHAIN_DIR - explicit override.
#   2. a standard install dir holding the required version (e.g. /usr/bin for the distro package).
#   3. arm-none-eabi-gcc from PATH.
# Pinning to a directory takes the compiler and every binutil from that one place.
if(NOT PX4_ARM_TOOLCHAIN_DIR AND DEFINED ENV{PX4_ARM_TOOLCHAIN_DIR})
	set(PX4_ARM_TOOLCHAIN_DIR "$ENV{PX4_ARM_TOOLCHAIN_DIR}")
endif()

if(NOT PX4_ARM_TOOLCHAIN_DIR AND PX4_ARM_TOOLCHAIN_ENFORCE)
	foreach(_dir /usr/bin /usr/local/bin /opt/homebrew/bin)
		if(EXISTS "${_dir}/${TOOLCHAIN_PREFIX}-gcc")
			execute_process(COMMAND "${_dir}/${TOOLCHAIN_PREFIX}-gcc" -dumpfullversion
				OUTPUT_VARIABLE _dir_gcc_version OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
			if(_dir_gcc_version VERSION_EQUAL PX4_ARM_TOOLCHAIN_VERSION)
				set(PX4_ARM_TOOLCHAIN_DIR "${_dir}")
				break()
			endif()
		endif()
	endforeach()
endif()

if(PX4_ARM_TOOLCHAIN_DIR)
	get_filename_component(PX4_ARM_TOOLCHAIN_DIR "${PX4_ARM_TOOLCHAIN_DIR}" ABSOLUTE)
	if(NOT EXISTS "${PX4_ARM_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}-gcc")
		message(FATAL_ERROR "PX4_ARM_TOOLCHAIN_DIR='${PX4_ARM_TOOLCHAIN_DIR}' does not contain ${TOOLCHAIN_PREFIX}-gcc")
	endif()
	set(TOOLCHAIN_BIN_PREFIX "${PX4_ARM_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}-")
else()
	set(TOOLCHAIN_BIN_PREFIX "${TOOLCHAIN_PREFIX}-")
endif()

set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN_PREFIX}gcc)
set(CMAKE_C_COMPILER_TARGET ${triple})

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_PREFIX}g++)
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN_PREFIX}gcc)

# needed for test compilation
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# compiler tools
find_program(CMAKE_AR ${TOOLCHAIN_BIN_PREFIX}gcc-ar)
find_program(CMAKE_GDB ${TOOLCHAIN_BIN_PREFIX}gdb)
find_program(CMAKE_LD ${TOOLCHAIN_BIN_PREFIX}ld)
find_program(CMAKE_LINKER ${TOOLCHAIN_BIN_PREFIX}ld)
find_program(CMAKE_NM ${TOOLCHAIN_BIN_PREFIX}gcc-nm)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_BIN_PREFIX}objcopy)
find_program(CMAKE_OBJDUMP ${TOOLCHAIN_BIN_PREFIX}objdump)
find_program(CMAKE_RANLIB ${TOOLCHAIN_BIN_PREFIX}gcc-ranlib)
find_program(CMAKE_STRIP ${TOOLCHAIN_BIN_PREFIX}strip)

# Enforce the resolved compiler version (fail loud on mismatch) unless disabled.
execute_process(
	COMMAND ${CMAKE_C_COMPILER} -dumpfullversion
	OUTPUT_VARIABLE GCC_VERSION
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
	RESULT_VARIABLE GCC_VERSION_RESULT)

if(NOT GCC_VERSION_RESULT EQUAL 0)
	message(FATAL_ERROR "could not run '${CMAKE_C_COMPILER}' to determine its version (is the arm-none-eabi toolchain installed / on PATH / PX4_ARM_TOOLCHAIN_DIR correct?)")
elseif(NOT PX4_ARM_TOOLCHAIN_ENFORCE)
	message(STATUS "arm-none-eabi-gcc ${GCC_VERSION} (version enforcement disabled)")
elseif(GCC_VERSION VERSION_EQUAL PX4_ARM_TOOLCHAIN_VERSION)
	message(STATUS "arm-none-eabi-gcc ${GCC_VERSION}")
else()
	message(FATAL_ERROR
		"arm-none-eabi-gcc ${GCC_VERSION} is not the supported version ${PX4_ARM_TOOLCHAIN_VERSION} (resolved '${CMAKE_C_COMPILER}').\n"
		"PX4 NuttX firmware must be built with exactly ${PX4_ARM_TOOLCHAIN_VERSION}, the same toolchain used by CI)."
		"If the right version is installed but not first on PATH, point the build at it:\n"
		"    export PX4_ARM_TOOLCHAIN_DIR=/path/to/toolchain/bin\n"
		"To build with a different compiler anyway, set PX4_ARM_TOOLCHAIN_VERSION=<version> (or =any to disable the check).")
endif()

set(CMAKE_FIND_ROOT_PATH get_file_component(${CMAKE_C_COMPILER} PATH))
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# os tools
foreach(tool grep make)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()
