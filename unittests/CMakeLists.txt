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

# add each test
# todo: add  mixer_test sbus2_test st24_test sf0x_test
add_executable(autodeclination_test autodeclination_test.cpp ${CMAKE_SOURCE_DIR}/../src/lib/geo_lookup/geo_mag_declination.c)
add_gtest(autodeclination_test)
