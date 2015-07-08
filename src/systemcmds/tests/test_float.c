/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file tests_float.c
 * Floating point tests
 */

#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "tests.h"
#include <math.h>
#include <float.h>

typedef union {
	float f;
	double d;
	uint8_t b[8];
} test_float_double_t;

int test_float(int argc, char *argv[])
{
	int ret = 0;

	printf("\n--- SINGLE PRECISION TESTS ---\n");
	printf("The single precision test involves calls to fabsf(),\nif test fails check this function as well.\n\n");

	float f1 = 1.55f;

	float sinf_zero = sinf(0.0f);
	float sinf_one = sinf(1.0f);
	float sqrt_two = sqrt(2.0f);

	if (fabsf(sinf_zero) < FLT_EPSILON) {
		printf("\t success: sinf(0.0f) == 0.0f\n");

	} else {
		printf("\t FAIL: sinf(0.0f) != 0.0f, result: %8.4f\n", (double)sinf_zero);
		ret = -4;
	}

	fflush(stdout);

	if (fabsf((sinf_one - 0.841470956802368164062500000000f)) < FLT_EPSILON) {
		printf("\t success: sinf(1.0f) == 0.84147f\n");

	} else {
		printf("\t FAIL: sinf(1.0f) != 0.84147f, result: %8.4f\n", (double)sinf_one);
		ret = -1;
	}

	fflush(stdout);

	float asinf_one = asinf(1.0f);

	if (fabsf((asinf_one - 1.570796251296997070312500000000f)) < FLT_EPSILON * 1.5f) {
		printf("\t success: asinf(1.0f) == 1.57079f\n");

	} else {
		printf("\t FAIL: asinf(1.0f) != 1.57079f, result: %f\n", (double)asinf_one);
		ret = -1;
	}

	fflush(stdout);

	float cosf_one = cosf(1.0f);

	if (fabsf((cosf_one - 0.540302336215972900390625000000f)) < FLT_EPSILON) {
		printf("\t success: cosf(1.0f) == 0.54030f\n");

	} else {
		printf("\t FAIL: cosf(1.0f) != 0.54030f, result: %8.4f\n", (double)cosf_one);
		ret = -1;
	}

	fflush(stdout);


	float acosf_one = acosf(1.0f);

	if (fabsf((acosf_one - 0.000000000000000000000000000000f)) < FLT_EPSILON) {
		printf("\t success: acosf(1.0f) == 0.0f\n");

	} else {
		printf("\t FAIL: acosf(1.0f) != 0.0f, result: %8.4f\n", (double)acosf_one);
		ret = -1;
	}

	fflush(stdout);


	float sinf_zero_one = sinf(0.1f);

	if (fabsf(sinf_zero_one - 0.0998334166f) < FLT_EPSILON) {
		printf("\t success: sinf(0.1f) == 0.09983f\n");

	} else {
		printf("\t FAIL: sinf(0.1f) != 0.09983f, result: %8.4f\n", (double)sinf_zero_one);
		ret = -2;
	}

	if (fabsf(sqrt_two - 1.41421356f) < FLT_EPSILON) {
		printf("\t success: sqrt(2.0f) == 1.41421f\n");

	} else {
		printf("\t FAIL: sqrt(2.0f) != 1.41421f, result: %8.4f\n", (double)sinf_zero_one);
		ret = -3;
	}

	float atan2f_ones = atan2f(1.0f, 1.0f);

	if (fabsf(atan2f_ones - 0.785398163397448278999490867136f) < 2.0f * FLT_EPSILON) {
		printf("\t success: atan2f(1.0f, 1.0f) == 0.78539f\n");

	} else {
		printf("\t FAIL: atan2f(1.0f, 1.0f) != 0.78539f, result: %8.4f\n", (double)atan2f_ones);
		ret = -4;
	}

	char sbuf[30];
	sprintf(sbuf, "%8.4f", (double)0.553415f);

	if (sbuf[0] == ' ' && sbuf[1] == ' ' && sbuf[2] == '0' &&
	    sbuf[3] == '.' && sbuf[4] == '5' && sbuf[5] == '5'
	    && sbuf[6] == '3' && sbuf[7] == '4' && sbuf[8] == '\0') {
		printf("\t success: printf(\"%%8.4f\", 0.553415f) == %8.4f\n", (double)0.553415f);

	} else {
		printf("\t FAIL: printf(\"%%8.4f\", 0.553415f) != \"  0.5534\", result: %s\n", sbuf);
		ret = -5;
	}

	sprintf(sbuf, "%8.4f", (double) - 0.553415f);

	if (sbuf[0] == ' ' && sbuf[1] == '-' && sbuf[2] == '0' &&
	    sbuf[3] == '.' && sbuf[4] == '5' && sbuf[5] == '5'
	    && sbuf[6] == '3' && sbuf[7] == '4' && sbuf[8] == '\0') {
		printf("\t success: printf(\"%%8.4f\", -0.553415f) == %8.4f\n", (double) - 0.553415f);

	} else {
		printf("\t FAIL: printf(\"%%8.4f\", -0.553415f) != \" -0.5534\", result: %s\n", sbuf);
		ret = -6;
	}





	printf("\n--- DOUBLE PRECISION TESTS ---\n");

	double d1 = 1.0111;
	double d2 = 2.0;

	double d1d2 = d1 * d2;

	if (fabs(d1d2 - 2.022200000000000219557705349871) < DBL_EPSILON) {
		printf("\t success: 1.0111 * 2.0 == 2.0222\n");

	} else {
		printf("\t FAIL: 1.0111 * 2.0 != 2.0222, result: %8.4f\n", d1d2);
		ret = -7;
	}

	fflush(stdout);

	// Assign value of f1 to d1
	d1 = f1;

	if (fabsf(f1 - (float)d1) < FLT_EPSILON) {
		printf("\t success: (float) 1.55f == 1.55 (double)\n");

	} else {
		printf("\t FAIL: (float) 1.55f != 1.55 (double), result: %8.4f\n", (double)f1);
		ret = -8;
	}

	fflush(stdout);


	double sin_zero = sin(0.0);
	double sin_one = sin(1.0);
	double atan2_ones = atan2(1.0, 1.0);

	if (fabs(sin_zero - 0.0) < DBL_EPSILON) {
		printf("\t success: sin(0.0) == 0.0\n");

	} else {
		printf("\t FAIL: sin(0.0) != 0.0, result: %8.4f\n", sin_zero);
		ret = -9;
	}

	if (fabs(sin_one - 0.841470984807896504875657228695) < DBL_EPSILON) {
		printf("\t success: sin(1.0) == 0.84147098480\n");

	} else {
		printf("\t FAIL: sin(1.0) != 1.0, result: %8.4f\n", sin_one);
		ret = -10;
	}

	if (fabs(atan2_ones - 0.785398163397448278999490867136) < 2.0 * DBL_EPSILON) {
		printf("\t success: atan2(1.0, 1.0) == 0.785398\n");

	} else {
		printf("\t FAIL: atan2(1.0, 1.0) != 0.785398, result: %8.4f\n", atan2_ones);
		ret = -11;
	}

	printf("\t testing pow() with magic value\n");
	printf("\t   (44330.0 * (1.0 - pow((96286LL / 101325.0), 0.190295)));\n");
	fflush(stdout);
	usleep(20000);
	double powres = (44330.0 * (1.0 - pow((96286LL / 101325.0), 0.190295)));
	printf("\t success: result: %8.4f\n", (double)powres);

	sprintf(sbuf, "%8.4f", 0.553415);

	if (sbuf[0] == ' ' && sbuf[1] == ' ' && sbuf[2] == '0' &&
	    sbuf[3] == '.' && sbuf[4] == '5' && sbuf[5] == '5'
	    && sbuf[6] == '3' && sbuf[7] == '4' && sbuf[8] == '\0') {
		printf("\t success: printf(\"%%8.4f\", 0.553415) == %8.4f\n", 0.553415);

	} else {
		printf("\t FAIL: printf(\"%%8.4f\", 0.553415) != \"  0.5534\", result: %s\n", sbuf);
		ret = -12;
	}

	sprintf(sbuf, "%8.4f", -0.553415);

	if (sbuf[0] == ' ' && sbuf[1] == '-' && sbuf[2] == '0' &&
	    sbuf[3] == '.' && sbuf[4] == '5' && sbuf[5] == '5'
	    && sbuf[6] == '3' && sbuf[7] == '4' && sbuf[8] == '\0') {
		printf("\t success: printf(\"%%8.4f\", -0.553415) == %8.4f\n", -0.553415);

	} else {
		printf("\t FAIL: printf(\"%%8.4f\", -0.553415) != \" -0.5534\", result: %s\n", sbuf);
		ret = -13;
	}


	if (ret == 0) {
		printf("\n SUCCESS: All float and double tests passed.\n");

	} else {
		printf("\n FAIL: One or more tests failed.\n");
	}

	printf("\n");

	return ret;
}
