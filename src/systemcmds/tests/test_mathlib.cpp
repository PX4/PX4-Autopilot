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

using namespace math;

const char* formatResult(bool res) {
	return res ? "OK" : "ERROR";
}

int test_mathlib(int argc, char *argv[])
{
	warnx("testing mathlib");

	Matrix3f m;
	m.identity();
	Matrix3f m1;
	Matrix<3,3> mq;
	mq.identity();
	Matrix<3,3> mq1;
	m1(0, 0) = 5.0;
	Vector3f v = Vector3f(1.0f, 2.0f, 3.0f);
	Vector3f v1;

	unsigned int n = 60000;

	hrt_abstime t0, t1;

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		v1 = m * v;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix * Vector: %s %.6fus", formatResult(v1 == v), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		mq1 = mq * mq;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix * Matrix: %s %.6fus", formatResult(mq1 == mq), (double)(t1 - t0) / n);
	mq1.dump();

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		m1 = m.transposed();
	}
	t1 = hrt_absolute_time();
	warnx("Matrix Transpose: %s %.6fus", formatResult(m1 == m), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		m1 = m.inversed();
	}
	t1 = hrt_absolute_time();
	warnx("Matrix Invert: %s %.6fus", formatResult(m1 == m), (double)(t1 - t0) / n);

	Matrix<4,4> mn;
	mn(0, 0) = 2.0f;
	mn(1, 0) = 3.0f;
	for (int i = 0; i < mn.getRows(); i++) {
		for (int j = 0; j < mn.getCols(); j++) {
			printf("%.3f ", mn(i, j));
		}
		printf("\n");
	}
	return 0;
}
