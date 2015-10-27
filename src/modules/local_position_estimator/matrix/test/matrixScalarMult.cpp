#include "Matrix.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
	float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	Matrix3f A(data);
	A = A * 2;
	float data_check[9] = {2, 4, 6, 8, 10, 12, 14, 16, 18};
	Matrix3f A_check(data_check);
	A.print();
	A_check.print();
	assert(A == A_check);
	return 0;
}
