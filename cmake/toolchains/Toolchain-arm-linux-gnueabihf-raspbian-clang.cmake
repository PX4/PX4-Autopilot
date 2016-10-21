include(CMakeForceCompiler)

if ($ENV{RPI_TOOLCHAIN_DIR} STREQUAL "")
        message(FATAL_ERROR "RPI_TOOLCHAIN_DIR not set")
else()
        set(RPI_TOOLCHAIN_DIR $ENV{RPI_TOOLCHAIN_DIR})
endif()

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
# requires a symbolic link typically from /usr/bin/clang
find_program(C_COMPILER clang
	PATHS ${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin
	NO_DEFAULT_PATH
	)

if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find C compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} Clang)

find_program(CXX_COMPILER clang++
	PATHS ${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin
	NO_DEFAULT_PATH
	)

if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find C++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} Clang)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-linux-gnueabihf-${tool}
		PATHS ${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find arm-linux-gnueabihf-${tool}")
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

add_definitions(
	-D __DF_RPI
	)

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
