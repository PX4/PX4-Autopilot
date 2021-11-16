#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	// general wraps
	TEST(fabs(wrap(4., 0., 10.) - 4.) < FLT_EPSILON);
	TEST(fabs(wrap(4., 0., 1.)) < FLT_EPSILON);
	TEST(fabs(wrap(-4., 0., 10.) - 6.) < FLT_EPSILON);
	TEST(fabs(wrap(-18., 0., 10.) - 2.) < FLT_EPSILON);
	TEST(fabs(wrap(-1.5, 3., 5.) - 4.5) < FLT_EPSILON);
	TEST(fabs(wrap(15.5, 3., 5.) - 3.5) < FLT_EPSILON);
	TEST(fabs(wrap(-1., 30., 40.) - 39.) < FLT_EPSILON);
	TEST(fabs(wrap(-8000., -555., 1.) - (-216.)) < FLT_EPSILON);
	TEST(fabs(wrap(0., 0., 360.)) < FLT_EPSILON);
	TEST(fabs(wrap(0. - FLT_EPSILON, 0., 360.) - (360. - FLT_EPSILON)) < FLT_EPSILON);
	TEST(fabs(wrap(0. + FLT_EPSILON, 0., 360.) - FLT_EPSILON) < FLT_EPSILON);
	TEST(fabs(wrap(360., 0., 360.)) < FLT_EPSILON);
	TEST(fabs(wrap(360. - FLT_EPSILON, 0., 360.) - (360. - FLT_EPSILON)) < FLT_EPSILON);
	TEST(fabs(wrap(360. + FLT_EPSILON, 0., 360.) - FLT_EPSILON) < FLT_EPSILON);

	// integer wraps
	TEST(wrap(-10, 0, 10) == 0);
	TEST(wrap(-4, 0, 10) == 6);
	TEST(wrap(0, 0, 10) == 0)
	TEST(wrap(4, 0, 10) == 4);
	TEST(wrap(10, 0, 10) == 0);

	// wrap pi
	TEST(fabs(wrap_pi(0.)) < FLT_EPSILON);
	TEST(fabs(wrap_pi(4.) - (4. - M_TWOPI)) < FLT_EPSILON);
	TEST(fabs(wrap_pi(-4.) - (-4. + M_TWOPI)) < FLT_EPSILON);
	TEST(fabs(wrap_pi(3.) - (3.)) < FLT_EPSILON);
	TEST(fabs(wrap_pi(100.) - (100. - 32. * M_PI)) < FLT_EPSILON);
	TEST(fabs(wrap_pi(-100.) - (-100. + 32. * M_PI)) < FLT_EPSILON);
	TEST(fabs(wrap_pi(-101.) - (-101. + 32. * M_PI)) < FLT_EPSILON);
	TEST(!is_finite(wrap_pi(NAN)));

	// wrap 2pi
	TEST(fabs(wrap_2pi(0.)) < FLT_EPSILON);
	TEST(fabs(wrap_2pi(-4.) - (-4. + 2. * M_PI)) < FLT_EPSILON);
	TEST(fabs(wrap_2pi(3.) - (3.)) < FLT_EPSILON);
	TEST(fabs(wrap_2pi(200.) - (200. - 31. * M_TWOPI)) < FLT_EPSILON);
	TEST(fabs(wrap_2pi(-201.) - (-201. + 32. * M_TWOPI)) < FLT_EPSILON);
	TEST(!is_finite(wrap_2pi(NAN)));

	// Equality checks
	TEST(isEqualF(1., 1.));
	TEST(!isEqualF(1., 2.));
	TEST(!isEqualF(NAN, 1.f));
	TEST(!isEqualF(1.f, NAN));
	TEST(!isEqualF(INFINITY, 1.f));
	TEST(!isEqualF(1.f, INFINITY));
	TEST(isEqualF(NAN, NAN));
	TEST(isEqualF(NAN, -NAN));
	TEST(isEqualF(-NAN, NAN));
	TEST(isEqualF(INFINITY, INFINITY));
	TEST(!isEqualF(INFINITY, -INFINITY));
	TEST(!isEqualF(-INFINITY, INFINITY));
	TEST(isEqualF(-INFINITY, -INFINITY));

	Vector3f a(1, 2, 3);
	Vector3f b(4, 5, 6);
	TEST(!isEqual(a, b));
	TEST(isEqual(a, a));

	Vector3f c(1, 2, 3);
	Vector3f d(1, 2, NAN);
	TEST(!isEqual(c, d));
	TEST(isEqual(c, c));
	TEST(isEqual(d, d));

	return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
