# defines:
#
# NM
# OBJCOPY
# LD
# CXX_COMPILER
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE

include(CMakeForceCompiler)

if ("$ENV{TOOLCHAIN_BIN_DIR}" STREQUAL "")
        message(FATAL_ERROR "TOOLCHAIN_BIN_DIR not set")
else()
        set(TOOLCHAIN_BIN_DIR $ENV{TOOLCHAIN_BIN_DIR})
endif()

if ("$ENV{CROSS_COMPILE}" STREQUAL "")
        message(FATAL_ERROR "CROSS_COMPILE not set")
else()
        set(CROSS_COMPILE $ENV{CROSS_COMPILE})
endif()

# 此处定义CROSS_COMPILE
set(CROSS_COMPILE $ENV{CROSS_COMPILE})

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER ${CROSS_COMPILE}-gcc)

if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find ${CROSS_COMPILE}-gcc compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER  ${CROSS_COMPILE}-g++)
if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find ${CROSS_COMPILE}-g++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${CROSS_COMPILE}-${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${CROSS_COMPILE}-${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo grep rm mkdir nm cp touch make unzip)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()

#add_definitions(
#	-D __DF_RPI
#)

set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})
set(CMAKE_C_FLAGS ${C_FLAGS})
set(CMAKE_CXX_LINKER_FLAGS ${C_FLAGS})

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
