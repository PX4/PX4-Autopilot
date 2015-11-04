if (COVERALLS)
	add_library(coverage
		coverage.cpp)
endif()

set(tests
	setIdentity
	inverse
	matrixMult
	vectorAssignment
	matrixAssignment
	matrixScalarMult
	transpose
	quaternion
	dcm
	vector
	vector3
	euler
	)

foreach(test ${tests})
	add_executable(${test}
		${test}.cpp)
	add_test(${test} ${test})
endforeach()
