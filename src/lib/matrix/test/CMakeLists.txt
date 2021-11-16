#include(gtest.cmake)

add_compile_options(
    -Wno-double-promotion
    -Wno-float-equal
)

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
    hatvee
    copyto
    least_squares
    upperRightTriangle
    dual
    pseudoInverse
)

add_custom_target(test_build)
foreach(test_name ${tests})
    add_executable(test-matrix_${test_name} ${test_name}.cpp)

    add_test(NAME test-matrix_${test_name}
        COMMAND test-matrix_${test_name}
        WORKING_DIRECTORY ${PX4_BINARY_DIR}
    )

    add_dependencies(test_build test-matrix_${test_name})
    add_dependencies(test_results test-matrix_${test_name})
endforeach()

px4_add_unit_gtest(SRC sparseVector.cpp)
