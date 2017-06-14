

set(extra_deps)
if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	find_package(px4 REQUIRED)
	find_package(mavlink_sitl_gazebo REQUIRED)

	add_rostest(mavros_offboard_attctl.test)
	add_rostest(mavros_offboard_posctl.test)

	add_rostest(mavros_mission_vtol_standard.test ARGS mission_file:=${CMAKE_CURRENT_SOURCE_DIR}/missions/vtol_new_1.txt)

	#catkin_add_nosetests(tests/test_your_node.py
        #             DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS})
	#  TIMEOUT
	set(extra_deps sitl_gazebo)
endif()


#=============================================================================
# unit tests
#

# TODO: find a way to keep this in sync with tests_main
set(tests
	autodeclination
	bson
	commander
	controllib
	conv
	dataman
	file2
	float
	gpio
	hrt
	hysteresis
	int
	mathlib
	matrix
	mavlink
	mc_pos_control
	mixer
	param
	parameters
	perf
	#rc # todo copy test_data or symlink for rc_test
	servo
	sf0x
	sleep
	uorb
	)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	list(REMOVE_ITEM tests
		hysteresis
		mixer
		uorb
	)
endif()

foreach(test_name ${tests})
	configure_file(templates/test_template.in ${CMAKE_CURRENT_BINARY_DIR}/test_${test_name}_generated)

	add_test(NAME ${test_name}
		COMMAND $<TARGET_FILE:px4> -d ${PX4_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/test_${test_name}_generated
		)

	set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "${test_name} FAILED")
	set_tests_properties(${test_name} PROPERTIES PASS_REGULAR_EXPRESSION "${test_name} PASSED")
endforeach()

add_custom_target(test_results
		COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test
		DEPENDS px4
		USES_TERMINAL
		COMMENT "Running tests in sitl"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results PROPERTIES EXCLUDE_FROM_ALL TRUE)

if (CMAKE_BUILD_TYPE STREQUAL Coverage)
	setup_target_for_coverage(test_coverage ${CMAKE_CTEST_COMMAND} coverage.info "--output-on-failure -T Test")
endif()

add_custom_target(test_results_junit
		COMMAND xsltproc ${PX4_SOURCE_DIR}/Tools/CTest2JUnit.xsl Testing/`head -n 1 < Testing/TAG`/Test.xml > JUnitTestResults.xml
		COMMENT "Converting ctest output to junit xml"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results_junit PROPERTIES EXCLUDE_FROM_ALL TRUE)

add_custom_target(test_cdash_submit
		COMMAND ${CMAKE_CTEST_COMMAND} -D Experimental
		USES_TERMINAL
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_cdash_submit PROPERTIES EXCLUDE_FROM_ALL TRUE)
