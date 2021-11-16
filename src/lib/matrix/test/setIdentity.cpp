#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	Matrix3f A;
	A.setIdentity();

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			if (i == j) {
				TEST(fabs(A(i, j) -  1) < FLT_EPSILON);

			} else {
				TEST(fabs(A(i, j) -  0) < FLT_EPSILON);
			}
		}
	}

	Matrix3f B;
	B.identity();

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			if (i == j) {
				TEST(fabs(B(i, j) -  1) < FLT_EPSILON);

			} else {
				TEST(fabs(B(i, j) -  0) < FLT_EPSILON);
			}
		}
	}

	return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
