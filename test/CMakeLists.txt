set(tests
	setIdentity
	inverse
	slice
	matrixMult
	vectorAssignment
	matrixAssignment
	matrixScalarMult
	transpose
	vector
	vector2
	vector3
	attitude
	filter
	integration
	squareMatrix
	helper
	)

add_custom_target(test_build)
foreach(test_name ${tests})
	add_executable(${test_name}
		${test_name}.cpp)
	add_test(test_${test_name} ${test_name})
	add_dependencies(test_build ${test_name})
endforeach()

if (${CMAKE_BUILD_TYPE} STREQUAL "Profile")
	add_custom_target(coverage
		COMMAND ${CMAKE_CTEST_COMMAND}
		COMMAND lcov --capture --directory . --output-file coverage.info
		COMMAND genhtml coverage.info --output-directory out
		COMMAND x-www-browser out/index.html
		WORKING_DIRECTORY ${CMAKE_BUILD_DIR}
		DEPENDS test_build
		)
endif()
