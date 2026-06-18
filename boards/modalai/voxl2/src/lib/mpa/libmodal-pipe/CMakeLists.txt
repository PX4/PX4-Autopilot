cmake_minimum_required(VERSION 3.22)
project(libmodal_pipe)


set(LIBNAME modal_pipe)


# Strawson's list of standard list of default gcc flags. Yes, I treat
# warnings as errors. Warnings exist to point out sloppy code and potential
# failure points for good reason. We do not approve of sloppy code.
set(CMAKE_C_FLAGS "-std=gnu99 -O3 -Werror -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=gnu++11 -O3 -Werror -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

# tell the linker not to worry about missing symbols in libraries
# core Modal AI libraries often have a lot of dependencies and this prevents
# higher level projects from needing to install so many dependencies at compile time
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")


## TODO REMOVE THIS ONCE CROSS 4.5 is released!! This directory is now in the
## voxl-cross 4.5 toolchain file!!!
if(PLATFORM STREQUAL QCS6490)
	link_directories(${TARGET_SYSROOT}/usr/lib/qcm6490)
endif()


set(EN_ION_BUF OFF CACHE BOOL "Whether to build with ion buf support or not")
if(EN_ION_BUF)
	add_definitions(-DEN_ION_BUF)
	message(STATUS "INFO: building with ion buf support")
else()
	message(STATUS "INFO: building without ion buf support")
endif()


# optionally build with mavlink support
set(ENABLE_MAVLINK_SUPPORT OFF CACHE BOOL "Whether to build with mavlink support or not")
if(ENABLE_MAVLINK_SUPPORT)
	add_definitions(-DEN_MAVLINK_SUPPORT)
	message(STATUS "INFO: building with mavlink support")
else()
	message(STATUS "INFO: building without mavlink support")
endif()


# add library subdirectory
add_subdirectory(library)

# optionally build example binaries
set(BUILD_EXAMPLES ON CACHE BOOL "Whether to build examples or not")
if(BUILD_EXAMPLES)
	message(STATUS "INFO: Building examples and tools")
	add_subdirectory(examples)
	add_subdirectory(tools)
else()
	message(STATUS "INFO: Skipping examples and tools")
endif()

