cmake_minimum_required(VERSION 2.8)

set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(VERSION_PATCH "2")

project(matrix CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    if(TESTING)
        set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
    else()
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
    endif()
    message(STATUS "set build type to ${CMAKE_BUILD_TYPE}")
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel;Coverage")

option(SUPPORT_STDIOSTREAM "If enabled provides support for << operator (as used with std::cout)" OFF)
option(TESTING "Enable testing" OFF)
option(FORMAT "Enable formatting" OFF)
option(COV_HTML "Display html for coverage" OFF)
option(ASAN "Enable address sanitizer" OFF)
option(UBSAN "Enable undefined behaviour sanitizer" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_SOURCE_DIR})

if(SUPPORT_STDIOSTREAM)
    add_definitions(-DSUPPORT_STDIOSTREAM)
endif()

if (("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" MATCHES "AppleClang"))
	set(CMAKE_CXX_FLAGS_COVERAGE
		"--coverage -ftest-coverage  -fdiagnostics-absolute-paths -O0 -fprofile-arcs -fno-inline-functions -fno-elide-constructors"
		CACHE STRING "Flags used by the C++ compiler during coverage builds" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
		"-ftest-coverage -fdiagnostics-absolute-paths"
		CACHE STRING "Flags used for linking binaries during coverage builds" FORCE)
else()
	set(CMAKE_CXX_FLAGS_COVERAGE
		# Add when GCC 9 or later is available as part of the default toolchain:  -fprofile-abs-path
		"--coverage -fprofile-arcs -ftest-coverage -O0 -fno-default-inline -fno-inline -fno-inline-small-functions -fno-elide-constructors"
		CACHE STRING "Flags used by the C++ compiler during coverage builds" FORCE)
	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
		"--coverage -ftest-coverage -lgcov"
		CACHE STRING "Flags used for linking binaries during coverage builds" FORCE)
endif()
mark_as_advanced(CMAKE_CXX_FLAGS_COVERAGE CMAKE_C_FLAGS_COVERAGE CMAKE_EXE_LINKER_FLAGS_COVERAGE)

add_compile_options(
    -pedantic
    -Wall
    -Warray-bounds
    -Wcast-align
    -Wcast-qual
    -Wconversion
    -Wctor-dtor-privacy
    -Wdisabled-optimization
    -Werror
    -Wextra
    -Wfloat-equal
    -Wformat-security
    -Wformat=2
    -Winit-self
    -Wlogical-op
    -Wmissing-declarations
    -Wmissing-include-dirs
    -Wno-sign-compare
    -Wno-unused
    -Wno-unused-parameter
    -Wnoexcept
    -Wold-style-cast
    -Woverloaded-virtual
    -Wpointer-arith
    -Wredundant-decls
    -Wreorder
    -Wshadow
    -Wsign-conversion
    -Wsign-promo
    -Wstrict-null-sentinel
    -Wswitch-default
    -Wundef
    -Wuninitialized
    -Wunused-variable
    )

# clang tolerate unknown gcc options
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_compile_options(
        -Wno-error=unused-command-line-argument-hard-error-in-future
        -Wno-unknown-warning-option
        )
else()
    add_compile_options(
        -Wstrict-overflow=5
        )
endif()

# santiziers (ASAN, UBSAN)
if(ASAN)
    message(STATUS "address sanitizer enabled")

    # environment variables
    #  ASAN_OPTIONS=detect_stack_use_after_return=1
    #  ASAN_OPTIONS=check_initialization_order=1

    add_compile_options(
        -fsanitize=address
        -g3
        -O1
        -fno-omit-frame-pointer
        )

    set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address)

elseif(UBSAN)
    message(STATUS "undefined behaviour sanitizer enabled")

    add_compile_options(
        -fsanitize=undefined
        -fsanitize=integer
        -g3
        -O1
        -fno-omit-frame-pointer
        )

    set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined)
endif()


add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

if(TESTING)
    enable_testing()
    add_subdirectory(test)
    add_dependencies(check test_build)

    add_custom_target(clang-tidy COMMAND clang-tidy -p . ${CMAKE_SOURCE_DIR}/test/*.cpp)
    add_dependencies(clang-tidy test_build)
endif()

if(FORMAT)
    set(astyle_exe ${CMAKE_BINARY_DIR}/astyle/src/bin/astyle)
    add_custom_command(OUTPUT ${astyle_exe}
        COMMAND wget http://sourceforge.net/projects/astyle/files/astyle/astyle%202.06/astyle_2.06_linux.tar.gz -O /tmp/astyle.tar.gz
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

    add_dependencies(check check_format)
endif()

set(CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
include(CPack)

# vim: set et fenc=utf-8 ft=cmake ff=unix sts=0 sw=4 ts=4 :
