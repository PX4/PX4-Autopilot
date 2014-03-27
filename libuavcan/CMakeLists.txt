cmake_minimum_required(VERSION 2.6)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Default build type is RelWithDebInfo" FORCE)
endif()

project(libuavcan)

#
# DSDSL compiler invocation
#
set(DSDLC_INPUTS "test/dsdl_test/root_ns_a" "test/dsdl_test/root_ns_b" "${CMAKE_SOURCE_DIR}/../dsdl/uavcan")
set(DSDLC_OUTPUT "include/dsdlc_generated")
add_custom_target(dsdlc dsdl_compiler/dsdlc.py ${DSDLC_INPUTS} -O${DSDLC_OUTPUT}
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
include_directories(${DSDLC_OUTPUT})

#
# -DUAVCAN_DEBUG=1 enables the tracing feature that writes debug info into stdout.
# Normally this feature should be used only for library development.
#
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O1 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -DUAVCAN_DEBUG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")

set(CPP11_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CPP03_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03 -Wno-variadic-macros -Wno-long-long")

include_directories(include)

#
# libuavcan
#
function(add_libuavcan name flags)
    file(GLOB_RECURSE LIBUAVCAN_CXX_FILES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp")
    add_library(${name} SHARED ${LIBUAVCAN_CXX_FILES})
    set_target_properties(${name} PROPERTIES COMPILE_FLAGS ${flags})
    add_dependencies(${name} dsdlc)
endfunction()

add_libuavcan(uavcan       ${CPP11_FLAGS})
add_libuavcan(uavcan_cpp03 ${CPP03_FLAGS})

# TODO installation rules

#
# Unit tests with gtest (optional)
#
function(add_test name library flags)
    find_package(Threads REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    file(GLOB_RECURSE TEST_CXX_FILES RELATIVE ${CMAKE_SOURCE_DIR} "test/*.cpp")
    add_executable(${name} ${TEST_CXX_FILES})
    add_dependencies(${name} ${library})

    set_target_properties(${name} PROPERTIES COMPILE_FLAGS ${flags})

    target_link_libraries(${name} ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    target_link_libraries(${name} ${CMAKE_BINARY_DIR}/lib${library}.so)
    target_link_libraries(${name} rt)

    # Tests run automatically upon successful build
    # If failing tests need to be investigated with debugger, use 'make --ignore-errors'
    add_custom_command(TARGET ${name} POST_BUILD
                       COMMAND ./${name} > "${name}.log"
                       WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endfunction()

find_package(GTest QUIET)
if (GTEST_FOUND)
    add_libuavcan(uavcan_optimized "-Wall -Wextra -Werror -pedantic -O3 -DNDEBUG -std=c++0x")
    add_test(libuavcan_test_optimized uavcan_optimized ${CPP11_FLAGS})

    add_test(libuavcan_test       uavcan       ${CPP11_FLAGS})
    add_test(libuavcan_test_cpp03 uavcan_cpp03 ${CPP03_FLAGS})
else (GTEST_FOUND)
    message(">> Google test library is not found, you will not be able to run tests")
endif (GTEST_FOUND)

#
# Static analysis with cppcheck (required), both library and unit test sources
#
add_custom_command(TARGET uavcan POST_BUILD
                   COMMAND ./cppcheck.sh
                   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
