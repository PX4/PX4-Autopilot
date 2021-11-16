#include "test_macros.hpp"

#include <matrix/math.hpp>

using namespace matrix;

int main()
{
	float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	Matrix3f A(data);
	A = A * 2;
	float data_check[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	Matrix3f A_check(data_check);
	TEST(isEqual(A, A_check));
	return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
