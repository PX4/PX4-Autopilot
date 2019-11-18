#include "test_macros.hpp"
#include <matrix/PseudoInverse.hpp>

using namespace matrix;

static const size_t n_large = 20;

int main()
{
    // 3x4 Matrix test
    float data0[12] = {
        0.f, 1.f,  2.f,  3.f,
        4.f, 5.f,  6.f,  7.f,
        8.f, 9.f, 10.f, 11.f
    };

    float data0_check[12] = {
        -0.3375f, -0.1f,  0.1375f,
        -0.13333333f, -0.03333333f,  0.06666667f,
        0.07083333f,  0.03333333f, -0.00416667f,
        0.275f,  0.1f, -0.075f
    };

    Matrix<float, 3, 4> A0(data0);
    Matrix<float, 4, 3> A0_I = geninv(A0);
    Matrix<float, 4, 3> A0_I_check(data0_check);

    TEST((A0_I - A0_I_check).abs().max() < 1e-5);

    // 4x3 Matrix test
    float data1[12] = {
        0.f, 4.f, 8.f,
        1.f, 5.f, 9.f,
        2.f, 6.f, 10.f,
        3.f, 7.f, 11.f
    };

    float data1_check[12] = {
        -0.3375f, -0.13333333f,  0.07083333f,  0.275f,
        -0.1f, -0.03333333f,  0.03333333f,  0.1f,
        0.1375f,  0.06666667f, -0.00416667f, -0.075f
    };

    Matrix<float, 4, 3> A1(data1);
    Matrix<float, 3, 4> A1_I = geninv(A1);
    Matrix<float, 3, 4> A1_I_check(data1_check);

    TEST((A1_I - A1_I_check).abs().max() < 1e-5);

    // Stess test
    Matrix<float, n_large, n_large - 1> A_large;
    A_large.setIdentity();
    Matrix<float, n_large - 1, n_large> A_large_I;

    for (size_t i = 0; i < n_large; i++) {
        A_large_I = geninv(A_large);
        TEST(isEqual(A_large, A_large_I.T()));
    }

    // Square matrix test
    float data2[9] = {0, 2, 3,
                      4, 5, 6,
                      7, 8, 10
                     };
    float data2_check[9] = {
        -0.4f, -0.8f,  0.6f,
        -0.4f,  4.2f, -2.4f,
        0.6f, -2.8f,  1.6f
    };

    SquareMatrix<float, 3> A2(data2);
    SquareMatrix<float, 3> A2_I = geninv(A2);
    SquareMatrix<float, 3> A2_I_check(data2_check);
    TEST((A2_I - A2_I_check).abs().max() < 1e-3);

    // Null matrix test
    Matrix<float, 6, 16> A3;
    Matrix<float, 16, 6> A3_I = geninv(A3);
    Matrix<float, 16, 6> A3_I_check;
    TEST((A3_I - A3_I_check).abs().max() < 1e-5);

    // Mock-up effectiveness matrix
    const float B_quad_w[6][16] = {
        {-0.5717536f,  0.43756646f,  0.5717536f, -0.43756646f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        { 0.35355328f, -0.35355328f,  0.35355328f, -0.35355328f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        { 0.28323701f,  0.28323701f, -0.28323701f, -0.28323701f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        { 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        { 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        {-0.25f, -0.25f, -0.25f, -0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
    };
    Matrix<float, 6, 16> B = Matrix<float, 6, 16>(B_quad_w);
    const float A_quad_w[16][6] = {
        { -0.495383f,  0.707107f,  0.765306f,  0.0f, 0.0f, -1.000000f },
        {  0.495383f, -0.707107f,  1.000000f,  0.0f, 0.0f, -1.000000f },
        {  0.495383f,  0.707107f, -0.765306f,  0.0f, 0.0f, -1.000000f },
        { -0.495383f, -0.707107f, -1.000000f,  0.0f, 0.0f, -1.000000f },
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
    };
    Matrix<float, 16, 6> A_check = Matrix<float, 16, 6>(A_quad_w);
    Matrix<float, 16, 6> A = geninv(B);
    TEST((A - A_check).abs().max() < 1e-5);

    // Test error case with erroneous rank in internal impl functions
    Matrix<float, 2, 2> L;
    Matrix<float, 2, 3> GM;
    Matrix<float, 3, 2> retM_check;
    Matrix<float, 3, 2> retM0 = GeninvImpl<float, 2, 3, 0>::genInvUnderdetermined(GM, L, 5);
    Matrix<float, 3, 2> GN;
    Matrix<float, 2, 3> retN_check;
    Matrix<float, 2, 3> retN0 = GeninvImpl<float, 3, 2, 0>::genInvOverdetermined(GN, L, 5);
    TEST((retM0 - retM_check).abs().max() < 1e-5);
    TEST((retN0 - retN_check).abs().max() < 1e-5);

    Matrix<float, 3, 2> retM1 = GeninvImpl<float, 2, 3, 1>::genInvUnderdetermined(GM, L, 5);
    Matrix<float, 2, 3> retN1 = GeninvImpl<float, 3, 2, 1>::genInvOverdetermined(GN, L, 5);
    TEST((retM1 - retM_check).abs().max() < 1e-5);
    TEST((retN1 - retN_check).abs().max() < 1e-5);

    float float_scale = 1.f;
    fullRankCholeskyTolerance(float_scale);
    double double_scale = 1.;
    fullRankCholeskyTolerance(double_scale);
    TEST(static_cast<double>(float_scale) > double_scale);

    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
