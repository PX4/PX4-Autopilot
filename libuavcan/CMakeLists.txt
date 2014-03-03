cmake_minimum_required(VERSION 2.6)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Default build type is RelWithDebInfo" FORCE)
endif()

project(libuavcan)

#
# -DUAVCAN_DEBUG=1 enables the tracing feature that writes debug info into stdout.
# Normally this feature should be used only for library development.
#
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O1 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -DUAVCAN_DEBUG=1 -DDEBUG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03 -Wall -Wextra -Werror -pedantic -Wno-variadic-macros")

set(CMAKE_C_FLAGS_RELWITHDEBINFO ${CMAKE_CXX_FLAGS_RELWITHDEBINFO})
set(CMAKE_C_FLAGS_RELEASE        ${CMAKE_CXX_FLAGS_RELEASE})
set(CMAKE_C_FLAGS_DEBUG          ${CMAKE_CXX_FLAGS_DEBUG})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra -Werror -pedantic")

include_directories(include)

#
# libuavcan
#
file(GLOB_RECURSE LIBUAVCAN_CXX_FILES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp")
file(GLOB_RECURSE LIBUAVCAN_C_FILES   RELATIVE ${CMAKE_SOURCE_DIR} "src/*.c")
add_library(uavcan SHARED ${LIBUAVCAN_CXX_FILES} ${LIBUAVCAN_C_FILES})

# TODO installation rules

#
# Unit tests with gtest (optional)
#
find_package(GTest QUIET)
if (GTEST_FOUND)
    find_package(Threads REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    file(GLOB_RECURSE TEST_CXX_FILES RELATIVE ${CMAKE_SOURCE_DIR} "test/*.cpp")
    add_executable(libuavcan_test ${TEST_CXX_FILES})
    add_dependencies(libuavcan_test uavcan)

    set_target_properties(libuavcan_test PROPERTIES
        COMPILE_FLAGS "-fno-rtti -fno-exceptions -Wno-unused-parameter -Wno-unused-function"
    )

    target_link_libraries(libuavcan_test ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    target_link_libraries(libuavcan_test ${CMAKE_BINARY_DIR}/libuavcan.so)

    # DSDL compiler invocation
    add_custom_target(dsdlc dsdl_compiler/dsdlc.py -v
                            test/dsdl_test/root_ns_a    # Input
                            -Otest/dsdlc_output         # Output
                            -Itest/dsdl_test/root_ns_b  # Include
                       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(libuavcan_test dsdlc)

    # Tests run automatically upon successful build
    # If failing tests need to be investigated with debugger, use 'make --ignore-errors'
    add_custom_command(TARGET libuavcan_test POST_BUILD
                       COMMAND ./libuavcan_test
                       WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
else (GTEST_FOUND)
    message(">> Google test library is not found, you will not be able to run tests")
endif (GTEST_FOUND)

#
# Static analysis with cppcheck (required), both library and unit test sources
#
add_custom_command(TARGET uavcan PRE_BUILD
                   COMMAND ./cppcheck.sh
                   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
