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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

set(GTEST_DIR gtest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)

set(PX_SRC ${CMAKE_SOURCE_DIR}/../src)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PX_SRC})
include_directories(${PX_SRC}/modules)
include_directories(${PX_SRC}/lib)
include_directories(${PX_SRC}/drivers)

add_definitions(-D__EXPORT=)
add_definitions(-D__PX4_TESTS)
add_definitions(-Dnoreturn_function=)
add_definitions(-Dmain_t=int)
add_definitions(-DERROR=-1)
add_definitions(-DOK=0)
add_definitions(-D_UNIT_TEST=)

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
add_executable(autodeclination_test autodeclination_test.cpp ${PX_SRC}/lib/geo_lookup/geo_mag_declination.c)
add_gtest(autodeclination_test)

# mixer_test
add_custom_command(OUTPUT ${PX_SRC}/modules/systemlib/mixer/mixer_multirotor.generated.h
                   COMMAND ${PX_SRC}/modules/systemlib/mixer/multi_tables.py > ${PX_SRC}/modules/systemlib/mixer/mixer_multirotor.generated.h)
add_executable(mixer_test mixer_test.cpp hrt.cpp
                          ${PX_SRC}/modules/systemlib/mixer/mixer.cpp
                          ${PX_SRC}/modules/systemlib/mixer/mixer_group.cpp
                          ${PX_SRC}/modules/systemlib/mixer/mixer_load.c
                          ${PX_SRC}/modules/systemlib/mixer/mixer_multirotor.cpp
                          ${PX_SRC}/modules/systemlib/mixer/mixer_multirotor.generated.h
                          ${PX_SRC}/modules/systemlib/mixer/mixer_simple.cpp
                          ${PX_SRC}/modules/systemlib/pwm_limit/pwm_limit.c
                          ${PX_SRC}/systemcmds/tests/test_mixer.cpp)
add_gtest(mixer_test)

# conversion_test
add_executable(conversion_test conversion_test.cpp ${PX_SRC}/systemcmds/tests/test_conv.cpp)
add_gtest(conversion_test)

# sbus2_test
add_executable(sbus2_test sbus2_test.cpp hrt.cpp)
add_gtest(sbus2_test)

# st24_test
add_executable(st24_test st24_test.cpp hrt.cpp ${PX_SRC}/lib/rc/st24.c)
add_gtest(st24_test)

# sumd_test
add_executable(sumd_test sumd_test.cpp hrt.cpp ${PX_SRC}/lib/rc/sumd.c)
add_gtest(sumd_test)

# sf0x_test
add_executable(sf0x_test sf0x_test.cpp ${PX_SRC}/drivers/sf0x/sf0x_parser.cpp)
add_gtest(sf0x_test)

# param_test
add_executable(param_test param_test.cpp
                          hrt.cpp
                          uorb_stub.cpp
                          ${PX_SRC}/modules/systemlib/param/param.c
                          ${PX_SRC}/modules/systemlib/bson/tinybson.c
                          )
add_gtest(param_test)
