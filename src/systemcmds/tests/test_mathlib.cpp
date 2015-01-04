/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file test_mathlib.cpp
 *
 * Mathlib test
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "tests.h"

#define TEST_OP(_title, _op) { unsigned int n = 60000; hrt_abstime t0, t1; t0 = hrt_absolute_time(); for (unsigned int j = 0; j < n; j++) { _op; }; t1 = hrt_absolute_time(); warnx(_title ": %.6fus", (double)(t1 - t0) / n); }

using namespace math;

int test_mathlib(int argc, char *argv[])
{
	int rc = 0;
	warnx("testing mathlib");

	{
		Vector<2> v;
		Vector<2> v1(1.0f, 2.0f);
		Vector<2> v2(1.0f, -1.0f);
		float data[2] = {1.0f, 2.0f};
		TEST_OP("Constructor Vector<2>()", Vector<2> v3);
		TEST_OP("Constructor Vector<2>(Vector<2>)", Vector<2> v3(v1));
		TEST_OP("Constructor Vector<2>(float[])", Vector<2> v3(data));
		TEST_OP("Constructor Vector<2>(float, float)", Vector<2> v3(1.0f, 2.0f));
		TEST_OP("Vector<2> = Vector<2>", v = v1);
		TEST_OP("Vector<2> + Vector<2>", v + v1);
		TEST_OP("Vector<2> - Vector<2>", v - v1);
		TEST_OP("Vector<2> += Vector<2>", v += v1);
		TEST_OP("Vector<2> -= Vector<2>", v -= v1);
		TEST_OP("Vector<2> * Vector<2>", v * v1);
		TEST_OP("Vector<2> %% Vector<2>", v1 % v2);
	}

	{
		Vector<3> v;
		Vector<3> v1(1.0f, 2.0f, 0.0f);
		Vector<3> v2(1.0f, -1.0f, 2.0f);
		float data[3] = {1.0f, 2.0f, 3.0f};
		TEST_OP("Constructor Vector<3>()", Vector<3> v3);
		TEST_OP("Constructor Vector<3>(Vector<3>)", Vector<3> v3(v1));
		TEST_OP("Constructor Vector<3>(float[])", Vector<3> v3(data));
		TEST_OP("Constructor Vector<3>(float, float, float)", Vector<3> v3(1.0f, 2.0f, 3.0f));
		TEST_OP("Vector<3> = Vector<3>", v = v1);
		TEST_OP("Vector<3> + Vector<3>", v + v1);
		TEST_OP("Vector<3> - Vector<3>", v - v1);
		TEST_OP("Vector<3> += Vector<3>", v += v1);
		TEST_OP("Vector<3> -= Vector<3>", v -= v1);
		TEST_OP("Vector<3> * float", v1 * 2.0f);
		TEST_OP("Vector<3> / float", v1 / 2.0f);
		TEST_OP("Vector<3> *= float", v1 *= 2.0f);
		TEST_OP("Vector<3> /= float", v1 /= 2.0f);
		TEST_OP("Vector<3> * Vector<3>", v * v1);
		TEST_OP("Vector<3> %% Vector<3>", v1 % v2);
		TEST_OP("Vector<3> length", v1.length());
		TEST_OP("Vector<3> length squared", v1.length_squared());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
		// Need pragma here intead of moving variable out of TEST_OP and just reference because
		// TEST_OP measures performance of vector operations.
		TEST_OP("Vector<3> element read", volatile float a = v1(0));
		TEST_OP("Vector<3> element read direct", volatile float a = v1.data[0]);
#pragma GCC diagnostic pop
		TEST_OP("Vector<3> element write", v1(0) = 1.0f);
		TEST_OP("Vector<3> element write direct", v1.data[0] = 1.0f);
	}

	{
		Vector<4> v;
		Vector<4> v1(1.0f, 2.0f, 0.0f, -1.0f);
		Vector<4> v2(1.0f, -1.0f, 2.0f, 0.0f);
		float data[4] = {1.0f, 2.0f, 3.0f, 4.0f};
		TEST_OP("Constructor Vector<4>()", Vector<4> v3);
		TEST_OP("Constructor Vector<4>(Vector<4>)", Vector<4> v3(v1));
		TEST_OP("Constructor Vector<4>(float[])", Vector<4> v3(data));
		TEST_OP("Constructor Vector<4>(float, float, float, float)", Vector<4> v3(1.0f, 2.0f, 3.0f, 4.0f));
		TEST_OP("Vector<4> = Vector<4>", v = v1);
		TEST_OP("Vector<4> + Vector<4>", v + v1);
		TEST_OP("Vector<4> - Vector<4>", v - v1);
		TEST_OP("Vector<4> += Vector<4>", v += v1);
		TEST_OP("Vector<4> -= Vector<4>", v -= v1);
		TEST_OP("Vector<4> * Vector<4>", v * v1);
	}

	{
		Vector<10> v1;
		v1.zero();
		float data[10];
		TEST_OP("Constructor Vector<10>()", Vector<10> v3);
		TEST_OP("Constructor Vector<10>(Vector<10>)", Vector<10> v3(v1));
		TEST_OP("Constructor Vector<10>(float[])", Vector<10> v3(data));
	}

	{
		Matrix<3, 3> m1;
		m1.identity();
		Matrix<3, 3> m2;
		m2.identity();
		Vector<3> v1(1.0f, 2.0f, 0.0f);
		TEST_OP("Matrix<3, 3> * Vector<3>", m1 * v1);
		TEST_OP("Matrix<3, 3> + Matrix<3, 3>", m1 + m2);
		TEST_OP("Matrix<3, 3> * Matrix<3, 3>", m1 * m2);
	}

	{
		Matrix<10, 10> m1;
		m1.identity();
		Matrix<10, 10> m2;
		m2.identity();
		Vector<10> v1;
		v1.zero();
		TEST_OP("Matrix<10, 10> * Vector<10>", m1 * v1);
		TEST_OP("Matrix<10, 10> + Matrix<10, 10>", m1 + m2);
		TEST_OP("Matrix<10, 10> * Matrix<10, 10>", m1 * m2);
	}

	{
		// test nonsymmetric +, -, +=, -=

		float data1[2][3] = {{1,2,3},{4,5,6}};
		float data2[2][3] = {{2,4,6},{8,10,12}};
		float data3[2][3] = {{3,6,9},{12,15,18}};
		
		Matrix<2, 3> m1(data1);
		Matrix<2, 3> m2(data2);
		Matrix<2, 3> m3(data3);

		if (m1 + m2 != m3) {
			warnx("Matrix<2, 3> + Matrix<2, 3> failed!");
			(m1 + m2).print();
			printf("!=\n");
			m3.print();
			rc = 1;
		}

		if (m3 - m2 != m1) {
			warnx("Matrix<2, 3> - Matrix<2, 3> failed!");
			(m3 - m2).print();
			printf("!=\n");
			m1.print();
			rc = 1;
		}

		m1 += m2;
		if (m1 != m3) {
			warnx("Matrix<2, 3> += Matrix<2, 3> failed!");
			m1.print();
			printf("!=\n");
			m3.print();
			rc = 1;
		}

		m1 -= m2;
		Matrix<2, 3> m1_orig(data1);
		if (m1 != m1_orig) {
			warnx("Matrix<2, 3> -= Matrix<2, 3> failed!");
			m1.print();
			printf("!=\n");
			m1_orig.print();
			rc = 1;
		}
	
	}

	return rc;
}
