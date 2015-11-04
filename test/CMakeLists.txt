add_library(instantiation
	instantiation.cpp
	)

link_libraries(instantiation)

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
