#include(gtest.cmake)

add_compile_options(
    -Wno-double-promotion
    -Wno-float-equal
)

set(tests
    slice
    vectorAssignment
    transpose
    vector
    vector2
    vector3
    squareMatrix
    upperRightTriangle
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

px4_add_unit_gtest(SRC MatrixAssignmentTest.cpp)
px4_add_unit_gtest(SRC MatrixAttitudeTest.cpp)
px4_add_unit_gtest(SRC MatrixCopyToTest.cpp)
px4_add_unit_gtest(SRC MatrixDualTest.cpp)
px4_add_unit_gtest(SRC MatrixFilterTest.cpp)
px4_add_unit_gtest(SRC MatrixHatveeTest.cpp)
px4_add_unit_gtest(SRC MatrixHelperTest.cpp)
px4_add_unit_gtest(SRC MatrixIntegralTest.cpp)
px4_add_unit_gtest(SRC MatrixInverseTest.cpp)
px4_add_unit_gtest(SRC MatrixLeastSquaresTest.cpp)
px4_add_unit_gtest(SRC MatrixMultiplicationTest.cpp)
px4_add_unit_gtest(SRC MatrixPseudoInverseTest.cpp)
px4_add_unit_gtest(SRC MatrixScalarMultiplicationTest.cpp)
px4_add_unit_gtest(SRC MatrixSetIdentityTest.cpp)
px4_add_unit_gtest(SRC MatrixSparseVectorTest.cpp)
px4_add_unit_gtest(SRC MatrixUnwrapTest.cpp)
