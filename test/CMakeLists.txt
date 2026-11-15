if(CMAKE_BUILD_TYPE STREQUAL "ThreadSanitizer")
	message(STATUS "TSAN build: skipping fuzztest, fetching GTest only")
	px4_setup_gtest_without_fuzztest()
else()
	px4_add_git_submodule(TARGET git_fuzztest PATH "fuzztest")
	message(STATUS "Adding fuzztest")
	# This will also add GTest
	add_subdirectory(fuzztest EXCLUDE_FROM_ALL)
endif()

# Ensure there's no -R without any filter expression since that trips newer ctest versions
if(TESTFILTER)
    set(TESTFILTERARG "-R")
else()
    set(TESTFILTERARG "")
endif()

add_custom_target(test_results
        # antlr4_tests_NOT_BUILT gets added by fuzztest
        COMMAND GTEST_COLOR=1 ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test
                ${TESTFILTERARG} ${TESTFILTER} --exclude-regex "antlr4_tests_NOT_BUILT"
        DEPENDS
        px4
        examples__dyn_hello
        USES_TERMINAL
        VERBATIM
        COMMENT "Running tests"
        WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Exercise the driver conversion with the real Fix/Fix2 types, without building the CAN driver.
set(gnss_dronecan_dir ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan)
set(gnss_dsdl_dir ${CMAKE_CURRENT_BINARY_DIR}/gnss_dsdl)
file(GLOB_RECURSE gnss_dsdl_inputs ${gnss_dronecan_dir}/dsdl/uavcan/*.uavcan)
add_custom_command(
    OUTPUT ${gnss_dsdl_dir}/uavcan/equipment/gnss/Fix.hpp ${gnss_dsdl_dir}/uavcan/equipment/gnss/Fix2.hpp
    COMMAND ${PYTHON_EXECUTABLE} ${gnss_dronecan_dir}/libuavcan/dsdl_compiler/libuavcan_dsdlc
        --outdir ${gnss_dsdl_dir} ${gnss_dronecan_dir}/dsdl/uavcan
    DEPENDS ${gnss_dsdl_inputs}
    COMMENT "Generating DroneCAN GNSS test types"
)
add_custom_target(gnss_test_dsdl DEPENDS
    ${gnss_dsdl_dir}/uavcan/equipment/gnss/Fix.hpp ${gnss_dsdl_dir}/uavcan/equipment/gnss/Fix2.hpp)
px4_add_unit_gtest(SRC ${PX4_SOURCE_DIR}/src/drivers/uavcan/sensors/GnssTimeTest.cpp
    INCLUDES ${gnss_dsdl_dir} ${gnss_dronecan_dir}/libuavcan/include
    COMPILE_FLAGS -DUAVCAN_CPP_VERSION=2003 -DUAVCAN_NO_GLOBAL_DATA_TYPE_REGISTRY=1)
add_dependencies(unit-GnssTime gnss_test_dsdl)
