#include "Matrix.hpp"
#include <assert.h>

using namespace matrix;

int main()
{
	Matrix3f A;
	A.setIdentity();
	assert(A.rows() == 3);
	assert(A.cols() == 3);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j) {
				assert(A(i, j) == 1);

			} else {
				assert(A(i, j) == 0);
			}
		}
	}

	return 0;
}
