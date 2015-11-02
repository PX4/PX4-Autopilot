set(tests
	setIdentity
	inverse
	matrixMult
	vectorAssignment
	matrixAssignment
	matrixScalarMult
	transpose
	)

foreach(test ${tests})
	add_executable(${test}
		${test}.cpp)
	add_test(${test} ${test})
endforeach()
