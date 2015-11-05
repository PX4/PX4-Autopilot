set(tests
	setIdentity
	inverse
	matrixMult
	vectorAssignment
	matrixAssignment
	matrixScalarMult
	transpose
	vector
	vector3
	attitude
	filter
	squareMatrix
	)

foreach(test ${tests})
	add_executable(${test}
		${test}.cpp)
	add_test(${test} ${test})
endforeach()
