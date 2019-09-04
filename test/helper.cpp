#include "test_macros.hpp"
#include <matrix/helper_functions.hpp>

using namespace matrix;

int main()
{
    // general wraps
    TEST(fabs(wrap(4.0, 0.0, 10.0) - 4.0) < FLT_EPSILON);
    TEST(fabs(wrap(4.0, 0.0, 1.0)) < FLT_EPSILON);
    TEST(fabs(wrap(-4.0, 0.0, 10.0) - 6.0) < FLT_EPSILON);
    TEST(fabs(wrap(-18.0, 0.0, 10.0) - 2.0) < FLT_EPSILON);
    TEST(fabs(wrap(-1.5, 3.0, 5.0) - 4.5) < FLT_EPSILON);
    TEST(fabs(wrap(15.5, 3.0, 5.0) - 3.5) < FLT_EPSILON);
    TEST(fabs(wrap(-1.0, 30.0, 40.0) - 39.0) < FLT_EPSILON);
    TEST(fabs(wrap(-8000.0, -555.0, 1.0) - (-216.0)) < FLT_EPSILON);
    TEST(!is_finite(wrap(1000.,0.,.01)));

    // wrap pi
    TEST(fabs(wrap_pi(4.0) - (4.0 - M_TWOPI)) < FLT_EPSILON);
    TEST(fabs(wrap_pi(-4.0) - (-4.0 + M_TWOPI)) < FLT_EPSILON);
    TEST(fabs(wrap_pi(3.0) - (3.0)) < FLT_EPSILON);
    TEST(fabs(wrap_pi(100.0f) - (100.0f - 32 * float(M_PI))) < 10e-5);
    TEST(fabs(wrap_pi(-100.0f) - (-100.0f + 32 * float(M_PI))) < 10e-5);
    TEST(fabs(wrap_pi(-101.0f) - (-101.0f + 32 * float(M_PI))) < 10e-5);
    TEST(!is_finite(wrap_pi(NAN)));

    // wrap 2pi
    TEST(fabs(wrap_2pi(-4.0) - (-4.0 + 2*M_PI)) < FLT_EPSILON);
    TEST(fabs(wrap_2pi(3.0) - (3.0)) < FLT_EPSILON);
    TEST(fabs(wrap_2pi(200.0f) - (200.0f - 31 * float(M_TWOPI))) < 10e-5);
    TEST(fabs(wrap_2pi(-201.0f) - (-201.0f + 32 * float(M_TWOPI))) < 10e-5);
    TEST(!is_finite(wrap_2pi(NAN)));

    Vector3f a(1, 2, 3);
    Vector3f b(4, 5, 6);
    TEST(!isEqual(a, b));
    TEST(isEqual(a, a));

    TEST(isEqualF(1.0f, 1.0f));
    TEST(!isEqualF(1.0f, 2.0f));
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
