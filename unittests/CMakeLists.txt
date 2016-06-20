cmake_minimum_required(VERSION 2.8)

include(CMakeForceCompiler)
#CMAKE_FORCE_C_COMPILER(clang Clang)
#CMAKE_FORCE_CXX_COMPILER(clang++ Clang)

if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
	add_compile_options(-Qunused-arguments )
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
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-exceptions -fno-rtti -fno-threadsafe-statics -D__CUSTOM_FILE_IO__ -D__PX4_UNIT_TESTS -g -Wall -Werror")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -fno-exceptions -fno-rtti -fno-threadsafe-statics -D__CUSTOM_FILE_IO__ -D__PX4_UNIT_TESTS -g -Wall -Werror")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -fsanitize=address -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare -Wno-unused-but-set-variable")

set(GTEST_DIR ${CMAKE_SOURCE_DIR}/googletest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)

set(PX4_SRC ${CMAKE_SOURCE_DIR}/../src)
set(PX4_SITL_BUILD ${PX4_SRC}/../build_posix_sitl_test)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PX4_SITL_BUILD}/src)
include_directories(${PX4_SITL_BUILD}/src/modules)
include_directories(${PX4_SITL_BUILD}/src/modules/param)
include_directories(${PX4_SITL_BUILD}/src/modules/uORB)
include_directories(${PX4_SRC})
include_directories(${PX4_SRC}/drivers)
include_directories(${PX4_SRC}/drivers/device)
include_directories(${PX4_SRC}/lib)
include_directories(${PX4_SRC}/lib/DriverFramework/framework/include)
include_directories(${PX4_SRC}/modules)
include_directories(${PX4_SRC}/modules/uORB)
include_directories(${PX4_SRC}/platforms)
include_directories(${PX4_SRC}/platforms/posix/include)
include_directories(${PX4_SRC}/platforms/posix/px4_layer)

add_definitions(-D__EXPORT=)
add_definitions(-D__PX4_POSIX)
add_definitions(-D__PX4_TESTS)
add_definitions(-D_UNIT_TEST=)
add_definitions(-DERROR=-1)
add_definitions(-Dmain_t=int)
add_definitions(-Dnoreturn_function=)
add_definitions(-DOK=0)

# check
add_custom_target(check
		COMMAND ${CMAKE_CTEST_COMMAND} -j2 --output-on-failure
		WORKING_DIR ${CMAKE_BINARY_DIR}
		USES_TERMINAL)

function(add_gtest)
	foreach(test_name ${ARGN})
		if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
			target_link_libraries(${test_name} gtest_main pthread px4_platform)
			add_definitions(-D__PX4_DARWIN)
		else()
			target_link_libraries(${test_name} gtest_main pthread rt px4_platform)
			add_definitions(-D__PX4_LINUX)
		endif()
		add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
		add_dependencies(check ${test_name})
	endforeach()
endfunction()

add_library(px4_platform 
			${PX4_SITL_BUILD}/src/modules/param/px4_parameters.c                         
			${PX4_SRC}/drivers/device/device_posix.cpp 
			${PX4_SRC}/drivers/device/i2c_posix.cpp 
			${PX4_SRC}/drivers/device/ringbuffer.cpp
			${PX4_SRC}/drivers/device/sim.cpp 
			${PX4_SRC}/drivers/device/vdev.cpp 
			${PX4_SRC}/drivers/device/vdev_posix.cpp
			${PX4_SRC}/drivers/device/vfile.cpp
			${PX4_SRC}/platforms/posix/px4_layer/drv_hrt.c
			${PX4_SRC}/platforms/posix/px4_layer/lib_crc32.c
			${PX4_SRC}/platforms/posix/px4_layer/px4_log.c
			${PX4_SRC}/platforms/posix/px4_layer/px4_log.c
			${PX4_SRC}/platforms/posix/px4_layer/px4_posix_impl.cpp
			${PX4_SRC}/platforms/posix/px4_layer/px4_posix_tasks.cpp
			${PX4_SRC}/platforms/posix/px4_layer/px4_sem.cpp
			${PX4_SRC}/platforms/posix/px4_layer/shmem_posix.c
			${PX4_SRC}/platforms/posix/work_queue/dq_addlast.c
			${PX4_SRC}/platforms/posix/work_queue/dq_rem.c
			${PX4_SRC}/platforms/posix/work_queue/dq_remfirst.c
			${PX4_SRC}/platforms/posix/work_queue/hrt_queue.c
			${PX4_SRC}/platforms/posix/work_queue/hrt_thread.c
			${PX4_SRC}/platforms/posix/work_queue/hrt_work_cancel.c
			${PX4_SRC}/platforms/posix/work_queue/queue.c
			${PX4_SRC}/platforms/posix/work_queue/sq_addafter.c
			${PX4_SRC}/platforms/posix/work_queue/sq_addlast.c
			${PX4_SRC}/platforms/posix/work_queue/sq_remfirst.c
			${PX4_SRC}/platforms/posix/work_queue/work_cancel.c
			${PX4_SRC}/platforms/posix/work_queue/work_lock.c
			${PX4_SRC}/platforms/posix/work_queue/work_queue.c
			${PX4_SRC}/platforms/posix/work_queue/work_thread.c
			)                           
target_include_directories(px4_platform PUBLIC ${PX4_SRC}/platforms)                           


#######################################################################
# TESTS
#######################################################################
#    add_executable(example_test example_test.cpp)
#    add_gtest(example_test)


# autodeclination_test
add_executable(autodeclination_test autodeclination_test.cpp ${PX4_SRC}/lib/geo_lookup/geo_mag_declination.c)
add_gtest(autodeclination_test)

# mixer_test
add_custom_command(OUTPUT ${PX4_SRC}/modules/systemlib/mixer/mixer_multirotor.generated.h
                   COMMAND ${PX4_SRC}/modules/systemlib/mixer/multi_tables.py > ${PX4_SRC}/modules/systemlib/mixer/mixer_multirotor.generated.h)
add_executable(mixer_test mixer_test.cpp
                          ${PX4_SRC}/modules/systemlib/mixer/mixer.cpp
                          ${PX4_SRC}/modules/systemlib/mixer/mixer_group.cpp
                          ${PX4_SRC}/modules/systemlib/mixer/mixer_load.c
                          ${PX4_SRC}/modules/systemlib/mixer/mixer_multirotor.cpp
                          ${PX4_SRC}/modules/systemlib/mixer/mixer_multirotor.generated.h
                          ${PX4_SRC}/modules/systemlib/mixer/mixer_simple.cpp
                          ${PX4_SRC}/modules/systemlib/pwm_limit/pwm_limit.c
                          ${PX4_SRC}/systemcmds/tests/test_mixer.cpp)
add_gtest(mixer_test)

# conversion_test
add_executable(conversion_test conversion_test.cpp ${PX4_SRC}/systemcmds/tests/test_conv.cpp)
add_gtest(conversion_test)

# sbus2_test
add_executable(sbus2_test sbus2_test.cpp
						${PX4_SRC}/lib/rc/sbus.c)
add_gtest(sbus2_test)

# DSM test
add_executable(dsm_test dsm_test.cpp
						${PX4_SRC}/lib/rc/dsm.c)
add_gtest(dsm_test)

# st24_test
add_executable(rc_input_test st24_test.cpp sumd_test.cpp
						${PX4_SRC}/lib/rc/st24.c
						${PX4_SRC}/lib/rc/sumd.c)
add_gtest(rc_input_test)

# sf0x_test
add_executable(sf0x_test sf0x_test.cpp
						${PX4_SRC}/drivers/sf0x/sf0x_parser.cpp)
add_gtest(sf0x_test)

# param_test
add_executable(param_test param_test.cpp uorb_stub.cpp
						${PX4_SRC}/modules/systemlib/bson/tinybson.c
						${PX4_SRC}/modules/systemlib/param/param.c)
target_link_libraries(param_test ${PX4_SITL_BUILD}/libmsg_gen.a)
add_gtest(param_test)

# param_shmem_test
#add_executable(param_shmem_test param_test.cpp uorb_stub.cpp
#						${PX4_SRC}/modules/systemlib/bson/tinybson.c
#						${PX4_SRC}/modules/systemlib/param/param_shmem.c)
#add_gtest(param_shmem_test)
