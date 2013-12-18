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

	Matrix<3,3> m3;
	m3.identity();
	Matrix<4,4> m4;
	m4.identity();
	Vector<3> v3;
	v3(0) = 1.0f;
	v3(1) = 2.0f;
	v3(2) = 3.0f;
	Vector<4> v4;
	v4(0) = 1.0f;
	v4(1) = 2.0f;
	v4(2) = 3.0f;
	v4(3) = 4.0f;
	Vector<3> vres3;
	Matrix<3,3> mres3;
	Matrix<4,4> mres4;

	Matrix3f m3old;
	m3old.identity();
	Vector3f v3old;
	v3old.x = 1.0f;
	v3old.y = 2.0f;
	v3old.z = 3.0f;
	Vector3f vres3old;
	Matrix3f mres3old;

	unsigned int n = 60000;

	hrt_abstime t0, t1;

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		vres3 = m3 * v3;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix3 * Vector3: %s %.6fus", formatResult(vres3 == v3), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		vres3old = m3old * v3old;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix3 * Vector3 OLD: %s %.6fus", formatResult(vres3old == v3old), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		mres3 = m3 * m3;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix3 * Matrix3: %s %.6fus", formatResult(mres3 == m3), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		mres3old = m3old * m3old;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix3 * Matrix3 OLD: %s %.6fus", formatResult(mres3old == m3old), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		mres4 = m4 * m4;
	}
	t1 = hrt_absolute_time();
	warnx("Matrix4 * Matrix4: %s %.6fus", formatResult(mres4 == m4), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		mres3 = m3.transposed();
	}
	t1 = hrt_absolute_time();
	warnx("Matrix3 Transpose: %s %.6fus", formatResult(mres3 == m3), (double)(t1 - t0) / n);

	t0 = hrt_absolute_time();
	for (unsigned int j = 0; j < n; j++) {
		mres3 = m3.inversed();
	}
	t1 = hrt_absolute_time();
	warnx("Matrix3 Invert: %s %.6fus", formatResult(mres3 == m3), (double)(t1 - t0) / n);
	return 0;
}
