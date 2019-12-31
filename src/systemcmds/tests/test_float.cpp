/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_float.cpp
 * Tests for floating point arithmetic.
 */

#include <unit_test.h>

#include <px4_platform_common/px4_config.h>

#include <float.h>
#include <math.h>

typedef union {
	float f;
	double d;
	uint8_t b[8];
} test_float_double_t;



class FloatTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool singlePrecisionTests();
	bool doublePrecisionTests();
};

bool FloatTest::singlePrecisionTests()
{
	float sinf_zero = sinf(0.0f);
	float sinf_one = sinf(1.0f);
	float sqrt_two = sqrtf(2.0f);

	ut_assert("sinf(0.0f) == 0.0f", fabsf(sinf_zero) < FLT_EPSILON);
	ut_assert("sinf(1.0f) == 0.84147f", fabsf((sinf_one - 0.841470956802368164062500000000f)) < FLT_EPSILON);

	float asinf_one = asinf(1.0f);
	ut_assert("asinf(1.0f) == 1.57079f", fabsf((asinf_one - 1.570796251296997070312500000000f)) < FLT_EPSILON * 1.5f);

	float cosf_one = cosf(1.0f);
	ut_assert("cosf(1.0f) == 0.54030f", fabsf((cosf_one - 0.540302336215972900390625000000f)) < FLT_EPSILON);

	float acosf_one = acosf(1.0f);
	ut_assert("acosf(1.0f) == 0.0f", fabsf((acosf_one - 0.000000000000000000000000000000f)) < FLT_EPSILON);

	float sinf_zero_one = sinf(0.1f);
	ut_assert("sinf(0.1f) == 0.09983f", fabsf(sinf_zero_one - 0.0998334166f) < FLT_EPSILON);

	ut_assert("sqrt(2.0f) == 1.41421f", fabsf(sqrt_two - 1.41421356f) < FLT_EPSILON);

	float atan2f_ones = atan2f(1.0f, 1.0f);
	ut_assert("atan2f(1.0f, 1.0f) == 0.78539f",
		  fabsf(atan2f_ones - 0.785398163397448278999490867136f) < 2.0f * FLT_EPSILON);

	char sbuf[30];
	sprintf(sbuf, "%8.4f", (double)0.553415f);
	ut_compare("sbuf[0]", sbuf[0], ' ');
	ut_compare("sbuf[1]", sbuf[1], ' ');
	ut_compare("sbuf[2]", sbuf[2], '0');
	ut_compare("sbuf[3]", sbuf[3], '.');
	ut_compare("sbuf[4]", sbuf[4], '5');
	ut_compare("sbuf[5]", sbuf[5], '5');
	ut_compare("sbuf[6]", sbuf[6], '3');
	ut_compare("sbuf[7]", sbuf[7], '4');
	ut_compare("sbuf[8]", sbuf[8], '\0');

	sprintf(sbuf, "%8.4f", (double) - 0.553415f);
	ut_compare("sbuf[0]", sbuf[0], ' ');
	ut_compare("sbuf[1]", sbuf[1], '-');
	ut_compare("sbuf[2]", sbuf[2], '0');
	ut_compare("sbuf[3]", sbuf[3], '.');
	ut_compare("sbuf[4]", sbuf[4], '5');
	ut_compare("sbuf[5]", sbuf[5], '5');
	ut_compare("sbuf[6]", sbuf[6], '3');
	ut_compare("sbuf[7]", sbuf[7], '4');
	ut_compare("sbuf[8]", sbuf[8], '\0');

	return true;
}


bool FloatTest::doublePrecisionTests()
{
	float f1 = 1.55f;

	double d1 = 1.0111;
	double d2 = 2.0;

	double d1d2 = d1 * d2;

	ut_assert("1.0111 * 2.0 == 2.0222", fabs(d1d2 - 2.022200000000000219557705349871) < DBL_EPSILON);

	// Assign value of f1 to d1
	d1 = f1;

	ut_assert("(float) 1.55f == 1.55 (double)", fabsf(f1 - (float)d1) < FLT_EPSILON);


	double sin_zero = sin(0.0);
	double sin_one = sin(1.0);
	double atan2_ones = atan2(1.0, 1.0);

	ut_assert("sin(0.0) == 0.0", fabs(sin_zero - 0.0) < DBL_EPSILON);
	ut_assert("sin(1.0) == 0.84147098480", fabs(sin_one - 0.841470984807896504875657228695) < DBL_EPSILON);
	ut_assert("atan2(1.0, 1.0) == 0.785398", fabs(atan2_ones - 0.785398163397448278999490867136) < 2.0 * DBL_EPSILON);
	ut_assert("testing pow() with magic value",
		  (44330.0 * (1.0 - pow((96286LL / 101325.0), 0.190295))) - 428.2293 <  DBL_EPSILON);


	char sbuf[30];
	sprintf(sbuf, "%8.4f", 0.553415);
	ut_compare("sbuf[0]", sbuf[0], ' ');
	ut_compare("sbuf[1]", sbuf[1], ' ');
	ut_compare("sbuf[2]", sbuf[2], '0');
	ut_compare("sbuf[3]", sbuf[3], '.');
	ut_compare("sbuf[4]", sbuf[4], '5');
	ut_compare("sbuf[5]", sbuf[5], '5');
	ut_compare("sbuf[6]", sbuf[6], '3');
	ut_compare("sbuf[7]", sbuf[7], '4');
	ut_compare("sbuf[8]", sbuf[8], '\0');


	sprintf(sbuf, "%8.4f", -0.553415);
	ut_compare("sbuf[0]", sbuf[0], ' ');
	ut_compare("sbuf[1]", sbuf[1], '-');
	ut_compare("sbuf[2]", sbuf[2], '0');
	ut_compare("sbuf[3]", sbuf[3], '.');
	ut_compare("sbuf[4]", sbuf[4], '5');
	ut_compare("sbuf[5]", sbuf[5], '5');
	ut_compare("sbuf[6]", sbuf[6], '3');
	ut_compare("sbuf[7]", sbuf[7], '4');
	ut_compare("sbuf[8]", sbuf[8], '\0');

	return true;
}

bool FloatTest::run_tests()
{
	ut_run_test(singlePrecisionTests);
	ut_run_test(doublePrecisionTests);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_float, FloatTest)
