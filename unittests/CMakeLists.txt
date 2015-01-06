cmake_minimum_required(VERSION 2.8)
project(unittests)
enable_testing()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(GTEST_DIR gtest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../src)
include_directories(${CMAKE_SOURCE_DIR}/../src/modules)
include_directories(${CMAKE_SOURCE_DIR}/../src/lib)

add_definitions(-D__EXPORT=)

# check
add_custom_target(unittests COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

function(add_gtest)
  foreach(test_name ${ARGN})
    target_link_libraries(${test_name} gtest_main)
    add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(unittests ${test_name})
  endforeach()
endfunction()


# add each test
add_executable(autodeclination_test autodeclination_test.cpp ${CMAKE_SOURCE_DIR}/../src/lib/geo_lookup/geo_mag_declination.c)
add_gtest(autodeclination_test)

# mixer_test
add_executable(mixer_test mixer_test.cpp hrt.cpp
                          ${CMAKE_SOURCE_DIR}/../src/modules/systemlib/mixer/mixer.cpp
                          ${CMAKE_SOURCE_DIR}/../src/modules/systemlib/mixer/mixer_group.cpp
                          ${CMAKE_SOURCE_DIR}/../src/modules/systemlib/mixer/mixer_load.c
                          ${CMAKE_SOURCE_DIR}/../src/modules/systemlib/mixer/mixer_multirotor.cpp
                          ${CMAKE_SOURCE_DIR}/../src/modules/systemlib/mixer/mixer_simple.cpp
                          ${CMAKE_SOURCE_DIR}/../src/modules/systemlib/pwm_limit/pwm_limit.c
                          ${CMAKE_SOURCE_DIR}/../src/systemcmds/tests/test_mixer.cpp)
add_gtest(mixer_test)

# conversion_test
add_executable(conversion_test conversion_test.cpp ${CMAKE_SOURCE_DIR}/../src/systemcmds/tests/test_conv.cpp)
add_gtest(conversion_test)

# sbus2_test
# TODO: move to gtest
add_executable(sbus2_test sbus2_test.cpp hrt.cpp)
add_gtest(sbus2_test)

# st24_test
# TODO: move to gtest
add_executable(st24_test st24_test.cpp hrt.cpp ${CMAKE_SOURCE_DIR}/../src/lib/rc/st24.c)
add_gtest(st24_test)

# sf0x_test
# TODO: move to gtest
add_executable(sf0x_test sf0x_test.cpp ${CMAKE_SOURCE_DIR}/../src/drivers/sf0x/sf0x_parser.cpp)
add_gtest(sf0x_test)
