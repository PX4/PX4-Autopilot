cmake_minimum_required(VERSION 2.8)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "1")
set(VERSION_PATCH "0")

project(matrix CXX)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
	message(STATUS "set build type to ${CMAKE_BUILD_TYPE}")
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
	STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Profile")

option(SUPPORT_STDIOSTREAM
	"If enabled provides support for << operator (as used with
	std::cout)" OFF)
option(TEST "Enable testing" OFF)
option(FORMAT "Enable formatting" OFF)

if(SUPPORT_STDIOSTREAM)
	add_definitions(-DSUPPORT_STDIOSTREAM)
endif()

set(CMAKE_CXX_FLAGS_PROFILE
	${CMAKE_CXX_FLAGS_DEBUG}
	--coverage
	-fno-inline
	-fno-inline-small-functions
	-fno-default-inline
	)
string(REPLACE ";" " " CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_PROFILE}")

set(CMAKE_CXX_FLAGS
	${CMAKE_CXX_FLAGS}
	-Wall
	-Wno-sign-compare
	-Wextra
	-Wshadow
	-Wfloat-equal
	-Wpointer-arith
	-Wmissing-declarations
	-Wno-unused-parameter
	-Werror=format-security
	-Werror=array-bounds
	#-Wfatal-errors
	-Werror=unused-variable
	-Werror=reorder
	-Werror=uninitialized
	-Werror=init-self
	-Wcast-qual
	-Wconversion
	-Wcast-align
	-Werror
	)
string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

enable_testing()

include_directories(${CMAKE_SOURCE_DIR})

file(GLOB_RECURSE COV_SRCS matrix/*.hpp matrix/*.cpp)

if(TEST)
	enable_testing()
	add_subdirectory(test)
endif()

if(FORMAT)
	set(astyle_exe ${CMAKE_BINARY_DIR}/astyle/src/bin/astyle)
	add_custom_command(OUTPUT ${astyle_exe}
		COMMAND wget http://sourceforge.net/projects/astyle/files/astyle/astyle%202.05.1/astyle_2.05.1_linux.tar.gz -O /tmp/astyle.tar.gz
		COMMAND tar -xvf /tmp/astyle.tar.gz
		COMMAND cd astyle/src && make -f ../build/gcc/Makefile
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		)

	add_custom_target(check_format
		COMMAND scripts/format.sh ${astyle_exe} 0
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		DEPENDS ${astyle_exe}
		VERBATIM
	)

	add_custom_target(format
		COMMAND scripts/format.sh ${astyle_exe} 1
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		VERBATIM
		DEPENDS ${astyle_exe}
	)
endif()

set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
include(CPack)
