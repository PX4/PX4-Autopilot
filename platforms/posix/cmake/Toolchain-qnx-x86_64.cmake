# QNX SDP 8.0 toolchain (qcc / q++) for x86_64 targets.
#
# Requires the QNX SDP environment (source qnxsdp-env.sh), which sets
# QNX_HOST and QNX_TARGET.

if(NOT DEFINED ENV{QNX_HOST} OR NOT DEFINED ENV{QNX_TARGET})
	message(FATAL_ERROR "QNX_HOST/QNX_TARGET not set. Source qnxsdp-env.sh first.")
endif()

set(QNX_HOST "$ENV{QNX_HOST}")
set(QNX_TARGET "$ENV{QNX_TARGET}")

set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_SYSTEM_VERSION 8.0.0)

set(arch gcc_ntox86_64)

set(CMAKE_C_COMPILER ${QNX_HOST}/usr/bin/qcc)
set(CMAKE_C_COMPILER_TARGET ${arch})

set(CMAKE_CXX_COMPILER ${QNX_HOST}/usr/bin/q++)
set(CMAKE_CXX_COMPILER_TARGET ${arch}_cxx)

set(CMAKE_ASM_COMPILER ${QNX_HOST}/usr/bin/qcc)
set(CMAKE_ASM_COMPILER_TARGET ${arch})

set(CMAKE_AR ${QNX_HOST}/usr/bin/ntox86_64-ar CACHE FILEPATH "archiver")
set(CMAKE_RANLIB ${QNX_HOST}/usr/bin/ntox86_64-ranlib CACHE FILEPATH "ranlib")
set(CMAKE_NM ${QNX_HOST}/usr/bin/ntox86_64-nm CACHE FILEPATH "nm")
set(CMAKE_OBJCOPY ${QNX_HOST}/usr/bin/ntox86_64-objcopy CACHE FILEPATH "objcopy")
set(CMAKE_STRIP ${QNX_HOST}/usr/bin/ntox86_64-strip CACHE FILEPATH "strip")
set(CMAKE_GDB ${QNX_HOST}/usr/bin/ntox86_64-gdb CACHE FILEPATH "gdb")

set(CMAKE_SYSROOT ${QNX_TARGET})
set(CMAKE_FIND_ROOT_PATH ${QNX_TARGET} ${QNX_TARGET}/x86_64)
# Host programs (python, make, grep) for code generation, target libs/headers.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# BOTH: nested ExternalProjects (Micro-XRCE-DDS) find packages they install in the build dir.
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# os tools used by the PX4 build
foreach(tool grep make)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()
