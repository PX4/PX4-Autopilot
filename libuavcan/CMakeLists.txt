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
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -DUAVCAN_DEBUG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03 -Wall -Wextra -Werror -pedantic -Wno-variadic-macros")

include_directories(include)

#
# libuavcan
#
file(GLOB_RECURSE LIBUAVCAN_CXX_FILES RELATIVE ${CMAKE_SOURCE_DIR} "src/*.cpp")
add_library(uavcan SHARED ${LIBUAVCAN_CXX_FILES})

# TODO installation rules

#
# Test
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

    add_custom_command(TARGET libuavcan_test POST_BUILD
                       COMMAND "./libuavcan_test"
                       WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
else (GTEST_FOUND)
    message(">> Google test is not found, you will not be able to run libuavcan_test")
endif (GTEST_FOUND)
