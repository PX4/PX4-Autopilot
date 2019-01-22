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
 * @file test_hysteresis.cpp
 * Tests for system timing hysteresis.
 */

#include <unit_test.h>
#include <unistd.h>

#include <systemlib/hysteresis/hysteresis.h>

class HysteresisTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool _init_false();
	bool _init_true();
	bool _zero_case();
	bool _change_after_time();
	bool _hysteresis_changed();
	bool _change_after_multiple_sets();
	bool _take_change_back();

	// timing on MacOS and Cygwin isn't great
#if defined(__PX4_DARWIN) ||  defined(__PX4_CYGWIN)
	static const int f = 10;
#else
	static const int f = 1;
#endif
};

bool HysteresisTest::run_tests()
{
	ut_run_test(_init_false);
	ut_run_test(_init_true);
	ut_run_test(_zero_case);
	ut_run_test(_change_after_time);
	ut_run_test(_hysteresis_changed);
	ut_run_test(_change_after_multiple_sets);
	ut_run_test(_take_change_back);

	return (_tests_failed == 0);
}

bool HysteresisTest::_init_false()
{
	systemlib::Hysteresis hysteresis(false);
	ut_assert_false(hysteresis.get_state());

	return true;
}

bool HysteresisTest::_init_true()
{
	systemlib::Hysteresis hysteresis(true);
	ut_assert_true(hysteresis.get_state());

	return true;
}

bool HysteresisTest::_zero_case()
{
	// Default is 0 hysteresis.
	systemlib::Hysteresis hysteresis(false);
	ut_assert_false(hysteresis.get_state());

	// Change and see result immediately.
	hysteresis.set_state_and_update(true);
	ut_assert_true(hysteresis.get_state());
	hysteresis.set_state_and_update(false);
	ut_assert_false(hysteresis.get_state());
	hysteresis.set_state_and_update(true);
	ut_assert_true(hysteresis.get_state());

	// A wait won't change anything.
	px4_usleep(1000 * f);
	hysteresis.update();
	ut_assert_true(hysteresis.get_state());

	return true;
}

bool HysteresisTest::_change_after_time()
{

	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);
	hysteresis.set_hysteresis_time_from(true, 3000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(4000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());
	px4_usleep(2000 * f);
	hysteresis.update();
	ut_assert_true(hysteresis.get_state());

	// Change back to false.
	hysteresis.set_state_and_update(false);
	ut_assert_true(hysteresis.get_state());
	px4_usleep(1000 * f);
	hysteresis.update();
	ut_assert_true(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());

	return true;
}

bool HysteresisTest::_hysteresis_changed()
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(true, 2000 * f);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	ut_assert_true(hysteresis.get_state());

	// Change hysteresis time.
	hysteresis.set_hysteresis_time_from(true, 10000 * f);

	// Change back to false.
	hysteresis.set_state_and_update(false);
	ut_assert_true(hysteresis.get_state());
	px4_usleep(7000 * f);
	hysteresis.update();
	ut_assert_true(hysteresis.get_state());
	px4_usleep(5000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());

	return true;
}

bool HysteresisTest::_change_after_multiple_sets()
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(true, 5000 * f);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(true);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(true);
	ut_assert_true(hysteresis.get_state());

	// Change to false.
	hysteresis.set_state_and_update(false);
	ut_assert_true(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(false);
	ut_assert_true(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.set_state_and_update(false);
	ut_assert_false(hysteresis.get_state());

	return true;
}

bool HysteresisTest::_take_change_back()
{
	systemlib::Hysteresis hysteresis(false);
	hysteresis.set_hysteresis_time_from(false, 5000 * f);

	// Change to true.
	hysteresis.set_state_and_update(true);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());
	// Change your mind to false.
	hysteresis.set_state_and_update(false);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(6000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());

	// And true again
	hysteresis.set_state_and_update(true);
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	ut_assert_false(hysteresis.get_state());
	px4_usleep(3000 * f);
	hysteresis.update();
	ut_assert_true(hysteresis.get_state());

	// The other directory is immediate.
	hysteresis.set_state_and_update(false);
	ut_assert_false(hysteresis.get_state());

	return true;
}

ut_declare_test_c(test_hysteresis, HysteresisTest)
