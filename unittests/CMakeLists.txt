cmake_minimum_required(VERSION 2.8)
project(unittests)
enable_testing()

set(GTEST_DIR gtest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../src)
include_directories(${CMAKE_SOURCE_DIR}/../src/modules)
include_directories(${CMAKE_SOURCE_DIR}/../src/lib)

add_definitions(-D__EXPORT=)
set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wno-write-strings")

# check
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

function(add_gtest)
  foreach(test_name ${ARGN})
    target_link_libraries(${test_name} gtest_main)
    add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(check ${test_name})
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
add_executable(sbus2_test sbus2_test.cpp hrt.cpp)


# todo: add st24_test sf0x_test
