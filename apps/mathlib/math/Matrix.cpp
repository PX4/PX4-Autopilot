/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Matrix.cpp
 *
 * matrix code
 */

#include "test/test.hpp"
#include <math.h>

#include "Matrix.hpp"

namespace math
{

static const float data_testA[] = {
	1, 2, 3,
	4, 5, 6
};
static Matrix testA(2, 3, data_testA);

static const float data_testB[] = {
	0, 1, 3,
	7, -1, 2
};
static Matrix testB(2, 3, data_testB);

static const float data_testC[] = {
	0, 1,
	2, 1,
	3, 2
};
static Matrix testC(3, 2, data_testC);

static const float data_testD[] = {
	0, 1, 2,
	2, 1, 4,
	5, 2, 0
};
static Matrix testD(3, 3, data_testD);

static const float data_testE[] = {
	1, -1, 2,
	0, 2, 3,
	2, -1, 1
};
static Matrix testE(3, 3, data_testE);

static const float data_testF[] = {
	3.777e006f, 2.915e007f, 0.000e000f,
	2.938e007f, 2.267e008f, 0.000e000f,
	0.000e000f, 0.000e000f, 6.033e008f
};
static Matrix testF(3, 3, data_testF);

int __EXPORT matrixTest()
{
	matrixAddTest();
	matrixSubTest();
	matrixMultTest();
	matrixInvTest();
	matrixDivTest();
	return 0;
}

int matrixAddTest()
{
	printf("Test Matrix Add\t\t: ");
	Matrix r = testA + testB;
	float data_test[] = {
		1.0f, 3.0f, 6.0f,
		11.0f, 4.0f, 8.0f
	};
	ASSERT(matrixEqual(Matrix(2, 3, data_test), r));
	printf("PASS\n");
	return 0;
}

int matrixSubTest()
{
	printf("Test Matrix Sub\t\t: ");
	Matrix r = testA - testB;
	float data_test[] = {
		1.0f, 1.0f, 0.0f,
		-3.0f, 6.0f, 4.0f
	};
	ASSERT(matrixEqual(Matrix(2, 3, data_test), r));
	printf("PASS\n");
	return 0;
}

int matrixMultTest()
{
	printf("Test Matrix Mult\t: ");
	Matrix r = testC * testB;
	float data_test[] = {
		7.0f, -1.0f,  2.0f,
		7.0f,  1.0f,  8.0f,
		14.0f,  1.0f, 13.0f
	};
	ASSERT(matrixEqual(Matrix(3, 3, data_test), r));
	printf("PASS\n");
	return 0;
}

int matrixInvTest()
{
	printf("Test Matrix Inv\t\t: ");
	Matrix origF = testF;
	Matrix r = testF.inverse();
	float data_test[] = {
		-0.0012518f,  0.0001610f, 0.0000000f,
		0.0001622f, -0.0000209f, 0.0000000f,
		0.0000000f,  0.0000000f, 1.6580e-9f
	};
	ASSERT(matrixEqual(Matrix(3, 3, data_test), r));
	// make sure F in unchanged
	ASSERT(matrixEqual(origF, testF));
	printf("PASS\n");
	return 0;
}

int matrixDivTest()
{
	printf("Test Matrix Div\t\t: ");
	Matrix r = testD / testE;
	float data_test[] = {
		0.2222222f, 0.5555556f, -0.1111111f,
		0.0f,       1.0f,         1.0,
		-4.1111111f, 1.2222222f,  4.5555556f
	};
	ASSERT(matrixEqual(Matrix(3, 3, data_test), r));
	printf("PASS\n");
	return 0;
}

bool matrixEqual(const Matrix &a, const Matrix &b, float eps)
{
	if (a.getRows() != b.getRows()) {
		printf("row number not equal a: %d, b:%d\n", a.getRows(), b.getRows());
		return false;

	} else if (a.getCols() != b.getCols()) {
		printf("column number not equal a: %d, b:%d\n", a.getCols(), b.getCols());
		return false;
	}

	bool ret = true;

	for (size_t i = 0; i < a.getRows(); i++)
		for (size_t j = 0; j < a.getCols(); j++) {
			if (!equal(a(i, j), b(i, j), eps)) {
				printf("element mismatch (%d, %d)\n", i, j);
				ret = false;
			}
		}

	return ret;
}

} // namespace math
