cmake_minimum_required(VERSION 2.8)

if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
	add_compile_options(-Qunused-arguments )
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	add_compile_options(-Qunused-arguments)
endif()

project(unittests)
enable_testing()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -std=gnu99 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++11 -g -fno-exceptions -fno-rtti -fno-threadsafe-statics")


set(GTEST_DIR ${CMAKE_SOURCE_DIR}/googletest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)

set(PX4_SRC ${CMAKE_SOURCE_DIR}/../src)
set(PX4_SITL_BUILD ${PX4_SRC}/../build_posix_sitl_default)

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
include_directories(${PX4_SRC}/platforms/posix/work_queue)

add_definitions(-D__CUSTOM_FILE_IO__)
add_definitions(-D__EXPORT=)
add_definitions(-D__PX4_POSIX)
add_definitions(-D__PX4_TESTS)
add_definitions(-D__PX4_UNIT_TESTS)
add_definitions(-D_UNIT_TEST=)
add_definitions(-DERROR=-1)
add_definitions(-Dmain_t=int)
add_definitions(-Dnoreturn_function=)
add_definitions(-DOK=0)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	add_definitions(-D__PX4_DARWIN)
else()
	add_definitions(-D__PX4_LINUX)
endif()

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
target_include_directories(px4_platform PUBLIC ${PX4_SRC}/platforms/posix/include)
target_include_directories(px4_platform PUBLIC ${PX4_SRC}/platforms/posix/work_queue)


# check
add_custom_target(check
		COMMAND ${CMAKE_CTEST_COMMAND} -j2 --output-on-failure
		WORKING_DIR ${CMAKE_BINARY_DIR}
		USES_TERMINAL)

# add_gtest
function(add_gtest)
	foreach(test_name ${ARGN})
		if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
			target_link_libraries(${test_name} gtest_main pthread px4_platform)
		else()
			target_link_libraries(${test_name} gtest_main pthread rt px4_platform)
		endif()
		add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
		add_dependencies(check ${test_name})
	endforeach()
endfunction()


#######################################################################
# TESTS
#######################################################################
#    add_executable(example_test example_test.cpp)
#    add_gtest(example_test)

# param_test
add_executable(param_test param_test.cpp uorb_stub.cpp
						${PX4_SRC}/modules/systemlib/bson/tinybson.c
						${PX4_SRC}/modules/systemlib/param/param.c)
target_link_libraries(param_test ${PX4_SITL_BUILD}/libmsg_gen.a)
add_gtest(param_test)
