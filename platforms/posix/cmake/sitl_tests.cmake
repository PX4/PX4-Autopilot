#=============================================================================
# tests
#

# TODO: find a way to keep this in sync with tests_main
set(tests
	autodeclination
	bson
	commander
	controllib
	conv
	ctlmath
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
	rc
	servo
	sf0x
	sleep
	uorb
	versioning
	)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	list(REMOVE_ITEM tests
		hysteresis
		mixer
		uorb
	)
endif()

foreach(test_name ${tests})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/tests_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/tests_${test_name}_generated)

	add_test(NAME ${test_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			posix-configs/SITL/init/test
			none
			none
			tests_${test_name}_generated
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR})

	set_tests_properties(${test_name} PROPERTIES FAIL_REGULAR_EXPRESSION "${test_name} FAILED")
	set_tests_properties(${test_name} PROPERTIES PASS_REGULAR_EXPRESSION "${test_name} PASSED")
endforeach()

# run arbitrary commands
set(test_cmds
	hello
	hrt_test
	muorb_test
	vcdev_test
	wqueue_test
	)

foreach(cmd_name ${test_cmds})
	configure_file(${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/cmd_template.in ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test/cmd_${cmd_name}_generated)

	add_test(NAME posix_${cmd_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			$<TARGET_FILE:px4>
			posix-configs/SITL/init/test
			none
			none
			cmd_${cmd_name}_generated
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR})

	set_tests_properties(posix_${cmd_name} PROPERTIES PASS_REGULAR_EXPRESSION "Shutting down")
endforeach()


add_custom_target(test_results
		COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -T Test
		DEPENDS px4
		USES_TERMINAL
		COMMENT "Running tests in sitl"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results PROPERTIES EXCLUDE_FROM_ALL TRUE)

if (CMAKE_BUILD_TYPE STREQUAL Coverage)
	setup_target_for_coverage(test_coverage "${CMAKE_CTEST_COMMAND} --output-on-failure -T Test" tests)
	setup_target_for_coverage(generate_coverage "${CMAKE_COMMAND} -E echo" generic)
endif()

add_custom_target(test_results_junit
		COMMAND xsltproc ${PX4_SOURCE_DIR}/Tools/CTest2JUnit.xsl Testing/`head -n 1 < Testing/TAG`/Test.xml > JUnitTestResults.xml
		DEPENDS test_results
		COMMENT "Converting ctest output to junit xml"
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_results_junit PROPERTIES EXCLUDE_FROM_ALL TRUE)

add_custom_target(test_cdash_submit
		COMMAND ${CMAKE_CTEST_COMMAND} -D Experimental
		USES_TERMINAL
		WORKING_DIRECTORY ${PX4_BINARY_DIR})
set_target_properties(test_cdash_submit PROPERTIES EXCLUDE_FROM_ALL TRUE)
