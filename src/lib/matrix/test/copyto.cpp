#include "test_macros.hpp"
#include <matrix/math.hpp>

using namespace matrix;

namespace
{
void doTheCopy(const Matrix<float, 2, 3> &A, float array_A[6])
{
	A.copyTo(array_A);
}
}

int main()
{
	float eps = 1e-6f;

	// Vector3 copyTo
	const Vector3f v(1, 2, 3);
	float dst3[3] = {};
	v.copyTo(dst3);

	for (size_t i = 0; i < 3; i++) {
		TEST(fabs(v(i) - dst3[i]) < eps);
	}

	// Quaternion copyTo
	Quatf q(1, 2, 3, 4);
	float dst4[4] = {};
	q.copyTo(dst4);

	for (size_t i = 0; i < 4; i++) {
		TEST(fabs(q(i) - dst4[i]) < eps);
	}

	// Matrix copyTo
	Matrix<float, 2, 3> A;
	A(0, 0) = 1;
	A(0, 1) = 2;
	A(0, 2) = 3;
	A(1, 0) = 4;
	A(1, 1) = 5;
	A(1, 2) = 6;
	float array_A[6] = {};
	doTheCopy(A, array_A);
	float array_row[6] = {1, 2, 3, 4, 5, 6};

	for (size_t i = 0; i < 6; i++) {
		TEST(fabs(array_A[i] - array_row[i]) < eps);
	}

	// Matrix copyToColumnMajor
	A.copyToColumnMajor(array_A);
	float array_column[6] = {1, 4, 2, 5, 3, 6};

	for (size_t i = 0; i < 6; i++) {
		TEST(fabs(array_A[i] - array_column[i]) < eps);
	}

	// Slice copyTo
	float dst5[2] = {};
	v.slice<2, 1>(0, 0).copyTo(dst5);

	for (size_t i = 0; i < 2; i++) {
		TEST(fabs(v(i) - dst5[i]) < eps);
	}

	float subarray_A[4] = {};
	A.slice<2, 2>(0, 0).copyToColumnMajor(subarray_A);
	float subarray_column[4] = {1, 4, 2, 5};

	for (size_t i = 0; i < 4; i++) {
		TEST(fabs(subarray_A[i] - subarray_column[i]) < eps);
	}

	return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
