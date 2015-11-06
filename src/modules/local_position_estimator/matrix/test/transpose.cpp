#include "Matrix.hpp"
#include <assert.h>
#include <stdio.h>

using namespace matrix;

int main()
{
	float data[9] = {1, 2, 3, 4, 5, 6};
	Matrix<float, 2, 3> A(data);
	Matrix<float, 3, 2> A_T = A.transpose();
	float data_check[9] = {1, 4, 2, 5, 3, 6};
	Matrix<float, 3, 2> A_T_check(data_check);
	A_T.print();
	A_T_check.print();
	assert(A_T == A_T_check);
	return 0;
}
