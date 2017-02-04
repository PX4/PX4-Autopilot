/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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
 * @file mc_pos_control_tests.cpp
 * Commander unit tests. Run the tests as follows:
 *   nsh> mc_pos_control_tests
 *
 */

#include <systemlib/err.h>
#include <unit_test/unit_test.h>
#include <mathlib/mathlib.h>

extern "C" __EXPORT int mc_pos_control_tests_main(int argc, char *argv[]);

bool mcPosControlTests();

//#include "../mc_pos_control_main.cpp"
class MulticopterPositionControl
{
public:
	bool		cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r,
					  const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res);
};

class McPosControlTests : public UnitTest
{
public:
	McPosControlTests();
	virtual ~McPosControlTests();

	virtual bool run_tests();

private:
	bool cross_sphere_line_test();
};

McPosControlTests::McPosControlTests()
{
}

McPosControlTests::~McPosControlTests()
{
}

bool McPosControlTests::cross_sphere_line_test()
{
	MulticopterPositionControl control = MulticopterPositionControl();

	math::Vector<3> prev = math::Vector<3>(0, 0, 0);
	math::Vector<3> curr = math::Vector<3>(0, 0, 2);
	math::Vector<3> res;
	bool retval = false;

	/*
	 * Testing 9 positions (+) around waypoints (o):
	 *
	 * Far             +              +              +
	 *
	 * Near            +              +              +
	 * On trajectory --+----o---------+---------o----+--
	 *                    prev                curr
	 *
	 * Expected targets (1, 2, 3):
	 * Far             +              +              +
	 *
	 *
	 * On trajectory -------1---------2---------3-------
	 *
	 *
	 * Near            +              +              +
	 * On trajectory -------o---1---------2-----3-------
	 *
	 *
	 * On trajectory --+----o----1----+--------2/3---+--
	 */

	// on line, near, before previous waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 0.0f, -0.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target A 0", res(0), 0.0f, 2);
	ut_compare_float("target A 1", res(1), 0.0f, 2);
	ut_compare_float("target A 2", res(2), 0.5f, 2);

	// on line, near, before target waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 0.0f, 1.0f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target B 0", res(0), 0.0f, 2);
	ut_compare_float("target B 1", res(1), 0.0f, 2);
	ut_compare_float("target B 2", res(2), 2.0f, 2);

	// on line, near, after target waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 0.0f, 2.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target C 0", res(0), 0.0f, 2);
	ut_compare_float("target C 1", res(1), 0.0f, 2);
	ut_compare_float("target C 2", res(2), 2.0f, 2);

	// near, before previous waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 0.5f, -0.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target D 0", res(0), 0.0f, 2);
	ut_compare_float("target D 1", res(1), 0.0f, 2);
	ut_compare_float("target D 2", res(2), 0.37f, 2);

	// near, before target waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 0.5f, 1.0f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target E 0", res(0), 0.0f, 2);
	ut_compare_float("target E 1", res(1), 0.0f, 2);
	ut_compare_float("target E 2", res(2), 1.87f, 2);

	// near, after target waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 0.5f, 2.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_true(retval);
	ut_compare_float("target F 0", res(0), 0.0f, 2);
	ut_compare_float("target F 1", res(1), 0.0f, 2);
	ut_compare_float("target F 2", res(2), 2.0f, 2);

	// far, before previous waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 2.0f, -0.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_false(retval);
	ut_compare_float("target G 0", res(0), 0.0f, 2);
	ut_compare_float("target G 1", res(1), 0.0f, 2);
	ut_compare_float("target G 2", res(2), 0.0f, 2);

	// far, before target waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 2.0f, 1.0f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_false(retval);
	ut_compare_float("target H 0", res(0), 0.0f, 2);
	ut_compare_float("target H 1", res(1), 0.0f, 2);
	ut_compare_float("target H 2", res(2), 1.0f, 2);

	// far, after target waypoint
	retval = control.cross_sphere_line(math::Vector<3>(0.0f, 2.0f, 2.5f), 1.0f, prev, curr, res);
	PX4_WARN("result %.2f, %.2f, %.2f", (double)res(0), (double)res(1), (double)res(2));
	ut_assert_false(retval);
	ut_compare_float("target I 0", res(0), 0.0f, 2);
	ut_compare_float("target I 1", res(1), 0.0f, 2);
	ut_compare_float("target I 2", res(2), 2.0f, 2);

	return true;
}

bool McPosControlTests::run_tests()
{
	ut_run_test(cross_sphere_line_test);

	return (_tests_failed == 0);
}

ut_declare_test(mcPosControlTests, McPosControlTests);

int mc_pos_control_tests_main(int argc, char *argv[])
{
	return mcPosControlTests() ? 0 : -1;
}
