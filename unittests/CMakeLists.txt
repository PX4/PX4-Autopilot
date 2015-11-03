cmake_minimum_required(VERSION 2.8)

include( CMakeForceCompiler )
#set( CMAKE_SYSTEM_NAME px4_posix_clang )
CMAKE_FORCE_C_COMPILER( clang Clang )
CMAKE_FORCE_CXX_COMPILER( clang++ Clang )
#set( CMAKE_C_COMPILER /opt/clang-3.4.2/bin/clang )
#set( CMAKE_CXX_COMPILER /opt/clang-3.4.2/bin/clang++ )
#set( CMAKE_FIND_ROOT_PATH /opt/clang-3.4.2/ )
#set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM_NEVER )
#set( CMAKE_FIND_ROOT_PATH_MODE_LIBARARY_ONLY )
#set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE_ONLY )

if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
  add_compile_options(-Qunused-arguments)
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_compile_options(-Qunused-arguments)
endif()

project(unittests)
enable_testing()


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-exceptions -fno-rtti -fno-threadsafe-statics -D__CUSTOM_FILE_IO__ -g -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fno-exceptions -fno-rtti -fno-threadsafe-statics -D__CUSTOM_FILE_IO__ -g -Wall")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -g")

#set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-isystem" )
#set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem" )

set(GTEST_DIR gtest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)

set(PX_SRC ${CMAKE_SOURCE_DIR}/../src)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PX_SRC})
include_directories(${PX_SRC}/modules)
include_directories(${PX_SRC}/modules/uORB)
include_directories(${PX_SRC}/lib)
include_directories(${PX_SRC}/drivers)
include_directories(${PX_SRC}/platforms)
include_directories(${PX_SRC}/platforms/posix/include)
include_directories(${PX_SRC}/platforms/posix/px4_layer)
include_directories(${PX_SRC}/drivers/device )


add_definitions(-D__EXPORT=)
add_definitions(-D__PX4_TESTS)
add_definitions(-Dnoreturn_function=)
add_definitions(-Dmain_t=int)
add_definitions(-DERROR=-1)
add_definitions(-DOK=0)
add_definitions(-D_UNIT_TEST=)
add_definitions(-D__PX4_POSIX)

# check
add_custom_target(unittests COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

function(add_gtest)
  foreach(test_name ${ARGN})
    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      target_link_libraries(${test_name} gtest_main pthread )
      add_definitions(-D__PX4_DARWIN)
    else()
      target_link_libraries(${test_name} gtest_main pthread rt )
      add_definitions(-D__PX4_LINUX)
    endif()
    add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(unittests ${test_name})
  endforeach()
endfunction()

add_library( px4_platform 
#                           ${PX_SRC}/platforms/common/px4_getopt.c
                           ${PX_SRC}/platforms/posix/px4_layer/px4_log.c
                           ${PX_SRC}/platforms/posix/px4_layer/px4_posix_impl.cpp
                           ${PX_SRC}/platforms/posix/px4_layer/px4_posix_tasks.cpp
                           ${PX_SRC}/platforms/posix/work_queue/work_lock.c
                           ${PX_SRC}/platforms/posix/work_queue/hrt_queue.c
                           ${PX_SRC}/platforms/posix/work_queue/work_queue.c
                           ${PX_SRC}/platforms/posix/work_queue/queue.c
                           ${PX_SRC}/platforms/posix/work_queue/work_cancel.c
                           ${PX_SRC}/platforms/posix/work_queue/hrt_work_cancel.c
                           ${PX_SRC}/platforms/posix/work_queue/hrt_thread.c
                           ${PX_SRC}/platforms/posix/work_queue/work_thread.c
                           ${PX_SRC}/platforms/posix/work_queue/dq_rem.c
                           ${PX_SRC}/platforms/posix/work_queue/sq_addlast.c
                           ${PX_SRC}/platforms/posix/work_queue/sq_addafter.c
                           ${PX_SRC}/platforms/posix/work_queue/dq_remfirst.c
                           ${PX_SRC}/platforms/posix/work_queue/sq_remfirst.c
                           ${PX_SRC}/platforms/posix/work_queue/dq_addlast.c
                           ${PX_SRC}/platforms/posix/px4_layer/lib_crc32.c
                           ${PX_SRC}/platforms/posix/px4_layer/drv_hrt.c
                           ${PX_SRC}/drivers/device/device_posix.cpp 
                           ${PX_SRC}/drivers/device/vdev.cpp 
                           ${PX_SRC}/drivers/device/vfile.cpp
                           ${PX_SRC}/drivers/device/vdev_posix.cpp
                           ${PX_SRC}/drivers/device/i2c_posix.cpp 
                           ${PX_SRC}/drivers/device/sim.cpp 
                           ${PX_SRC}/drivers/device/ringbuffer.cpp                           
 )
                           
#target_include_directories( px4_platform PUBLIC ${PX_SRC}/platforms )                           
                           
                           

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
target_link_libraries( mixer_test px4_platform )

                          
add_gtest(mixer_test)

# conversion_test
add_executable(conversion_test conversion_test.cpp ${PX_SRC}/systemcmds/tests/test_conv.cpp)
target_link_libraries( conversion_test px4_platform )
add_gtest(conversion_test)

# sbus2_test
add_executable(sbus2_test sbus2_test.cpp hrt.cpp)
target_link_libraries( sbus2_test px4_platform )
add_gtest(sbus2_test)

# st24_test
add_executable(st24_test st24_test.cpp hrt.cpp ${PX_SRC}/lib/rc/st24.c)
target_link_libraries( st24_test px4_platform )
add_gtest(st24_test)

# sumd_test
add_executable(sumd_test sumd_test.cpp hrt.cpp ${PX_SRC}/lib/rc/sumd.c)
target_link_libraries( sumd_test px4_platform )
add_gtest(sumd_test)

# sf0x_test
add_executable(sf0x_test sf0x_test.cpp ${PX_SRC}/drivers/sf0x/sf0x_parser.cpp)
target_link_libraries( sf0x_test px4_platform )
add_gtest(sf0x_test)

# param_test
add_executable(param_test param_test.cpp
                          hrt.cpp
                          uorb_stub.cpp
                          ${PX_SRC}/modules/systemlib/param/param.c
                          ${PX_SRC}/modules/systemlib/bson/tinybson.c
                          )
target_link_libraries( param_test px4_platform )                          
                          
add_gtest(param_test)

# uorb test
add_executable(uorb_tests uorb_unittests/uORBCommunicator_gtests.cpp
                          uorb_unittests/uORBCommunicatorMock.cpp
                          uorb_unittests/uORBCommunicatorMockLoopback.cpp
                          ${PX_SRC}/modules/uORB/uORBDevices_posix.cpp
                          ${PX_SRC}/modules/uORB/uORBManager_posix.cpp
                          ${PX_SRC}/modules/uORB/objects_common.cpp
                          ${PX_SRC}/modules/uORB/uORBUtils.cpp
                          ${PX_SRC}/modules/uORB/uORB.cpp
                          )
target_link_libraries( uorb_tests px4_platform )
                          
add_gtest(uorb_tests)
