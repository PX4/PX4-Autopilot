/****************************************************************************
 *
 *  Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file test_bezierQuad.cpp
 * Test for Bezier curve computation.
 */

#include <unit_test.h>
#include <float.h>
#include <stdlib.h>
#include <time.h>

#include "../../lib/bezier/BezierQuad.hpp"

class BezierQuadTest : public UnitTest
{
public:
	virtual bool run_tests();

private:

	bool _get_states_from_time();
	bool _get_arc_length();
	bool _set_bez_from_vel();

	float random(float min, float max);

};


bool BezierQuadTest::run_tests()
{
	ut_run_test(_get_states_from_time);
	ut_run_test(_get_arc_length);
	ut_run_test(_set_bez_from_vel);

	return (_tests_failed == 0);
}

bool BezierQuadTest::_get_states_from_time()
{
	// symmetric around 0
	matrix::Vector3f pt0(-0.5f, 0.0f, 0.0f);
	matrix::Vector3f ctrl(0.0f, 0.5f, 0.0f);
	matrix::Vector3f pt1(0.5f, 0.0f, 0.0f);

	// create bezier with default t = [0,1]
	bezier::BezierQuad_f bz(pt0, ctrl, pt1);

	matrix::Vector3f pos, vel, acc;
	float precision = 0.00001;

	// states at time = 0
	bz.getStates(pos, vel, acc, 0.0f);

	ut_compare_float("pos[0] not equal pt0[0]", pos(0), pt0(0), precision);
	ut_compare_float("pos[1] not equal pt0[1]", pos(1), pt0(1), precision);
	ut_compare_float("pos[2] not equal pt0[2]", pos(2), pt0(2), precision);

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal 1", vel(1), 1.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 0", acc(0), 0.0f, precision);
	ut_compare_float("acc not equal 1", acc(1), -2.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 1
	bz.getStates(pos, vel, acc, 1.0f);

	ut_compare_float("pos[0] not equal pt1[0]", pos(0), pt1(0), precision);
	ut_compare_float("pos[1] not equal pt1[1]", pos(1), pt1(1), precision);
	ut_compare_float("pos[2] not equal pt1[2]", pos(2), pt1(2), precision);

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal -1", vel(1), -1.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 0", acc(0), 0.0f, precision);
	ut_compare_float("acc not equal 1", acc(1), -2.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 0.5
	bz.getStates(pos, vel, acc, 0.50f);

	// pos must be equal to ctrl(0) and lower than ctrl(1)
	ut_compare_float("pos[0] not equal ctrl[0]", pos(0), ctrl(0), precision);
	ut_assert_true(pos(1) < ctrl(1));

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal -1", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 0", acc(0), 0.0f, precision);
	ut_compare_float("acc not equal -2", acc(1), -2.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// acceleration
	pt0 = matrix::Vector3f(0.0f, 0.0f, 0.0f);
	ctrl = matrix::Vector3f(0.0f, 0.0f, 0.0f);
	pt1 = matrix::Vector3f(1.0f, 0.0f, 0.0f);

	// create bezier with default t = [0,1]
	bz.setBezier(pt0, ctrl, pt1, 1.0f);

	// states at time = 0.0
	bz.getStates(pos, vel, acc, 0.0f);

	ut_compare_float("pos[0] not equal pt0[0]", pos(0), pt0(0), precision);
	ut_compare_float("pos[1] not equal pt0[1]", pos(1), pt0(1), precision);
	ut_compare_float("pos[2] not equal pt0[2]", pos(2), pt0(2), precision);

	ut_compare_float("slope not equal 0", vel(0), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 2", acc(0), 2.0f, precision);
	ut_compare_float("acc not equal 0", acc(1), 0.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 1.0
	bz.getStates(pos, vel, acc, 1.0f);

	ut_compare_float("pos[0] not equal pt1[0]", pos(0), pt1(0), precision);
	ut_compare_float("pos[1] not equal pt1[1]", pos(1), pt1(1), precision);
	ut_compare_float("pos[2] not equal pt1[2]", pos(2), pt1(2), precision);

	ut_compare_float("slope not equal 2", vel(0), 2.0f, precision);
	ut_compare_float("slope not equal 0", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 2", acc(0), 2.0f, precision);
	ut_compare_float("acc not equal 0", acc(1), 0.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	// states at time = 0.5
	bz.getStates(pos, vel, acc, 0.5f);

	ut_compare_float("slope not equal 1", vel(0), 1.0f, precision);
	ut_compare_float("slope not equal 0", vel(1), 0.0f, precision);
	ut_compare_float("slope not equal 0", vel(2), 0.0f, precision);

	ut_compare_float("acc not equal 2", acc(0), 2.0f, precision);
	ut_compare_float("acc not equal 0", acc(1), 0.0f, precision);
	ut_compare_float("acc not equal 0", acc(2), 0.0f, precision);

	return true;

}

bool BezierQuadTest::_get_arc_length()
{
	// create random numbers
	srand(0); // choose a constant to make it deterministic

	float min = -50.f;
	float max = 50.f;
	float resolution = 0.1f;

	matrix::Vector3f pt0, pt1, ctrl;
	float duration, arc_length, triangle_length, straigth_length;
	float T = 100.0f;

	// loop trough different control points 100x and check if arc_length is in the expected range
	for (int i = 0; i < 100 ; i++) {
		// random bezier point
		pt0 = matrix::Vector3f(random(min, max), random(min, max), random(min, max));
		pt1 = matrix::Vector3f(random(min, max), random(min, max), random(min, max));
		ctrl = matrix::Vector3f(random(min, max), random(min, max), random(min, max));

		// use for each test a new duration
		duration = random(0.0f, T);

		// create bezier
		bezier::BezierQuad_f bz(pt0, ctrl, pt1, duration);

		// compute arc length, triangle length and straigh length
		arc_length = bz.getArcLength(resolution);
		triangle_length = (ctrl - pt0).length() + (pt1 - ctrl).length();
		straigth_length = (pt1 - pt0).length();

		// we also compute length from going point to point and add segment
		float time_increment = duration / T;
		float t = 0.0f + time_increment;
		matrix::Vector3f p0 = pt0;
		float sum_segments = 0.0f;

		for (int s = 0; s < (int)T; s++) {
			matrix::Vector3f nextpt = bz.getPoint(t);
			sum_segments = (nextpt - p0).length() + sum_segments;
			p0 = bz.getPoint(t);
			t = t + time_increment;
		}

		// test comparisons
		ut_assert_true((triangle_length >= arc_length) && (arc_length >= straigth_length)
			       && (fabsf(arc_length - sum_segments) < 1.f));
	}


	return true;
}

bool BezierQuadTest::_set_bez_from_vel()
{
	// create random numbers
	srand(100); // choose a constant to make it deterministic

	float low = -50.0f;
	float max = 50.0f;
	float precision = 0.001f;

	for (int i = 0; i < 20; i++) {

		// set velocity
		matrix::Vector3f ctrl(random(low, max), random(low, max), random(low, max));
		matrix::Vector3f vel0(random(low, max), random(low, max), random(low, max));
		matrix::Vector3f vel1(random(low, max), random(low, max), random(low, max));
		float duration = random(0.0f, 100.0f);

		bezier::BezierQuad_f bz;;
		bz.setBezFromVel(ctrl, vel0, vel1, duration);

		// get velocity back
		matrix::Vector3f v0 = bz.getVelocity(0.0f);
		matrix::Vector3f v1 = bz.getVelocity(duration);
		ut_compare_float("", vel0(0), v0(0), precision);
		ut_compare_float("", vel1(0), v1(0), precision);

		ut_compare_float("", vel0(1), v0(1), precision);
		ut_compare_float("", vel1(1), v1(1), precision);

		ut_compare_float("", vel0(2), v0(2), precision);
		ut_compare_float("", vel1(2), v1(2), precision);
	}

	return true;
}

float BezierQuadTest::random(float min, float max)
{
	float s = rand() / (float)RAND_MAX;
	return (min + s * (max - min));

}

ut_declare_test_c(test_bezierQuad, BezierQuadTest)
