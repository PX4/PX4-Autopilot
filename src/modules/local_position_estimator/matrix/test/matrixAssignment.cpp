#include "Matrix.hpp"
#include <assert.h>

using namespace matrix;

int main()
{
	Matrix3f m;
	m.setZero();
	m(0, 0) = 1;
	m(0, 1) = 2;
	m(0, 2) = 3;
	m(1, 0) = 4;
	m(1, 1) = 5;
	m(1, 2) = 6;
	m(2, 0) = 7;
	m(2, 1) = 8;
	m(2, 2) = 9;

	m.print();

	float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	Matrix3f m2(data);
	m2.print();

	assert(m == m2);

	return 0;
}
