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
 * @file Vector.cpp
 *
 * math vector
 */

#include "test/test.hpp"

#include "Vector.hpp"

namespace math
{

static const float data_testA[] = {1, 3};
static const float data_testB[] = {4, 1};

static Vector testA(2, data_testA);
static Vector testB(2, data_testB);

int __EXPORT vectorTest()
{
	vectorAddTest();
	vectorSubTest();
	return 0;
}

int vectorAddTest()
{
	printf("Test Vector Add\t\t: ");
	Vector r = testA + testB;
	float data_test[] = {5.0f, 4.0f};
	ASSERT(vectorEqual(Vector(2, data_test), r));
	printf("PASS\n");
	return 0;
}

int vectorSubTest()
{
	printf("Test Vector Sub\t\t: ");
	Vector r(2);
	r = testA - testB;
	float data_test[] = { -3.0f, 2.0f};
	ASSERT(vectorEqual(Vector(2, data_test), r));
	printf("PASS\n");
	return 0;
}

bool vectorEqual(const Vector &a, const Vector &b, float eps)
{
	if (a.getRows() != b.getRows()) {
		printf("row number not equal a: %d, b:%d\n", a.getRows(), b.getRows());
		return false;
	}

	bool ret = true;

	for (size_t i = 0; i < a.getRows(); i++) {
		if (!equal(a(i), b(i), eps)) {
			printf("element mismatch (%d)\n", i);
			ret = false;
		}
	}

	return ret;
}

} // namespace math
