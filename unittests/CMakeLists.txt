cmake_minimum_required(VERSION 3.1)

if("${CMAKE_C_COMPILER_ID}" MATCHES "Clang")
	add_compile_options(-Qunused-arguments )
endif()
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
	add_compile_options(-Qunused-arguments)
endif()

project(px4_unittests)
enable_testing()

message(STATUS ${CONFIG})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -std=gnu99 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=gnu++0x -g -fno-exceptions -fno-rtti -fno-threadsafe-statics -DCONFIG_WCHAR_BUILTIN -D__CUSTOM_FILE_IO__")

# code coverage
if ($ENV{PX4_CODE_COVERAGE} MATCHES "1")
	message(STATUS "Code coverage build flags enabled")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3 -fprofile-arcs -ftest-coverage --coverage")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g3 -fprofile-arcs -ftest-coverage --coverage")
endif()

if (NOT PX4_SOURCE_DIR)
	set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()

set(GTEST_DIR ${PX4_SOURCE_DIR}/unittests/googletest)
add_subdirectory(${GTEST_DIR})
include_directories(${GTEST_DIR}/include)

set(PX4_SRC ${PX4_SOURCE_DIR}/src)
set(PX4_SITL_BUILD ${PX4_SRC}/../build_posix_sitl_default)

include_directories(${PX4_SOURCE_DIR})
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
include_directories(${PX4_SRC}/include)
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
add_definitions(-DPARAM_NO_AUTOSAVE)
add_definitions(-DERROR=-1)
add_definitions(-Dmain_t=int)
add_definitions(-Dnoreturn_function=)
add_definitions(-DOK=0)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	add_definitions(-D__PX4_DARWIN)
else()
	add_definitions(-D__PX4_LINUX)
endif()

# the following flags only apply to the PX4 source
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include visibility.h")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include visibility.h")

set(PX4_PLATFORM
	${PX4_SITL_BUILD}/libmsg_gen.a
	${PX4_SITL_BUILD}/src/drivers/boards/sitl/libdrivers__boards__sitl.a
	${PX4_SITL_BUILD}/src/drivers/device/libdrivers__device.a
	${PX4_SITL_BUILD}/src/lib/DriverFramework/framework/liblib__DriverFramework__framework.a
	${PX4_SITL_BUILD}/src/lib/DriverFramework/framework/src/libdf_driver_framework.a
	${PX4_SITL_BUILD}/src/platforms/common/libplatforms__common.a
	${PX4_SITL_BUILD}/src/platforms/posix/px4_layer/libplatforms__posix__px4_layer.a
	${PX4_SITL_BUILD}/src/platforms/posix/work_queue/libplatforms__posix__work_queue.a
	)

# check
add_custom_target(check
		COMMAND ${CMAKE_CTEST_COMMAND} -j2 --output-on-failure
		WORKING_DIR ${PX4_BINARY_DIR}
		USES_TERMINAL)

# add_gtest
function(add_gtest)
	foreach(test_name ${ARGN})
		if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
			target_link_libraries(${test_name} gtest_main pthread ${PX4_PLATFORM})
		else()
			target_link_libraries(${test_name} gtest_main pthread rt ${PX4_PLATFORM})
		endif()
		add_test(NAME ${test_name} COMMAND ${test_name} --gtest_output=xml WORKING_DIRECTORY ${PX4_BINARY_DIR})
		add_dependencies(check ${test_name})
	endforeach()
endfunction()


#######################################################################
# TESTS
#######################################################################
#    add_executable(example_test example_test.cpp)
#    add_gtest(example_test)

# param_test
add_executable(param_test setup.cpp param_test.cpp uorb_stub.cpp
						${PX4_SRC}/modules/systemlib/bson/tinybson.c
						${PX4_SRC}/modules/systemlib/param/param.c)
target_link_libraries(param_test ${PX4_PLATFORM})
add_gtest(param_test)
