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

function(add_gtest)
  foreach(test_name ${ARGN})
    target_link_libraries(${test_name} gtest_main)
    add_test(${test_name}Test ${test_name})
  endforeach()
endfunction()


# add each test
# todo: add  mixer_test sbus2_test st24_test sf0x_test
add_executable(autodeclination_test autodeclination_test.cpp ${CMAKE_SOURCE_DIR}/../src/lib/geo_lookup/geo_mag_declination.c)
add_gtest(autodeclination_test)
