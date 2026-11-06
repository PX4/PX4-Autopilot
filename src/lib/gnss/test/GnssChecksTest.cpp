/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>

#include "../gnss_checks.hpp"

class GnssChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		GnssChecks::Params params{};
		params.min_health_time_us = 1'000'000;
		setParams(params);

		_sample.lat = 47.0;
		_sample.lon = 9.0;
		_sample.alt = 800.f;
		_sample.vel_valid = true;
		_sample.hacc = 1.f;
		_sample.vacc = 2.f;
		_sample.sacc = 0.2f;
		_sample.fix_type = 3;
		_sample.nsats = 10;
		_sample.pdop = 1.f;
	}

	void setParams(GnssChecks::Params params)
	{
		_checks.setParams(params);
	}

	void runSeconds(float duration_s, bool armed, bool in_air)
	{
		for (int k = 0; k < static_cast<int>(duration_s * 10.f); k++) {
			_sample.time_us += 100'000;
			_checks.run(_sample, armed, in_air, !in_air);
		}
	}

	// Fly armed in the air at the reported velocity, while the reported position moves at that velocity plus pos_error_rate
	void flySeconds(float duration_s, const matrix::Vector3f &vel, const matrix::Vector3f &pos_error_rate = {})
	{
		const MapProjection origin(47.0, 9.0);

		for (int k = 0; k < static_cast<int>(duration_s * 10.f); k++) {
			_sample.time_us += 100'000;
			_pos_ned += (vel + pos_error_rate) * 0.1f;
			origin.reproject(_pos_ned(0), _pos_ned(1), _sample.lat, _sample.lon);
			_sample.alt = 800.f - _pos_ned(2);
			_sample.vel = vel;
			_checks.run(_sample, true, true, false);
		}
	}

	GnssChecks _checks;
	gnssChecksSample _sample{};
	matrix::Vector3f _pos_ned{};
};

TEST_F(GnssChecksTest, passesAfterHealthTime)
{
	// WHEN: a receiver passes the strict checks on the ground
	runSeconds(0.5f, false, false);

	// THEN: it passes once they held for the health time
	EXPECT_FALSE(_checks.passed());

	runSeconds(1.f, false, false);

	EXPECT_TRUE(_checks.passed());
	EXPECT_TRUE(_checks.strict());
}

TEST_F(GnssChecksTest, relaxedOnlyAfterStrictPass)
{
	// GIVEN: a receiver that starts publishing in flight, within the relaxed thresholds only
	_sample.hacc = 10.f;

	// WHEN: it publishes for longer than the health time
	runSeconds(5.f, true, true);

	// THEN: the strict checks still apply, and it doesn't pass
	EXPECT_FALSE(_checks.passed());
	EXPECT_TRUE(_checks.strict());

	// WHEN: it passes the strict checks for the health time
	_sample.hacc = 1.f;
	runSeconds(1.5f, true, true);

	// THEN: it passes, and the relaxed checks apply from then on
	EXPECT_TRUE(_checks.passed());
	EXPECT_TRUE(_checks.strict());

	_sample.hacc = 10.f;
	runSeconds(1.f, true, true);

	EXPECT_TRUE(_checks.passed());
	EXPECT_FALSE(_checks.strict());
}

TEST_F(GnssChecksTest, relaxedWhileArmedOnGround)
{
	// GIVEN: a receiver that passed the strict checks, on a vehicle that armed and hasn't taken off
	runSeconds(2.f, false, false);
	runSeconds(1.f, true, false);

	// WHEN: its accuracy degrades beyond the strict threshold but within the relaxed one
	_sample.hacc = 10.f;
	runSeconds(1.f, true, false);

	// THEN: it still passes
	EXPECT_TRUE(_checks.passed());
	EXPECT_FALSE(_checks.strict());
	EXPECT_FALSE(_checks.passedStrict());
}

TEST_F(GnssChecksTest, strictFailureOfOneSample)
{
	// GIVEN: a receiver that passed the strict checks, then took off
	runSeconds(2.f, false, false);
	runSeconds(1.f, true, true);

	// WHEN: a single sample has too few satellites for the strict checks
	_sample.nsats = 4;
	runSeconds(0.1f, true, true);

	// THEN: that sample doesn't pass the strict checks, but still passes the relaxed ones that decide
	EXPECT_FALSE(_checks.passedStrict());
	EXPECT_TRUE(_checks.passed());
	EXPECT_EQ(_checks.getStrictFailFlags(), vehicle_gnss_s::CHECK_NSATS);
	EXPECT_EQ(_checks.getRelaxedFailFlags(), 0);

	// WHEN: the next sample has enough again
	_sample.nsats = 10;
	runSeconds(0.1f, true, true);

	// THEN: it passes them again at once, as only a failure of the relaxed checks restarts the required pass duration
	EXPECT_TRUE(_checks.passedStrict());
}

TEST_F(GnssChecksTest, strictInFlight)
{
	// GIVEN: a receiver that passed the strict checks, then took off
	runSeconds(2.f, false, false);
	runSeconds(1.f, true, true);

	EXPECT_TRUE(_checks.passed());
	EXPECT_TRUE(_checks.strict());
	EXPECT_TRUE(_checks.passedStrict());

	// WHEN: its accuracy degrades beyond the strict threshold but within the relaxed one
	_sample.hacc = 10.f;
	runSeconds(1.f, true, true);

	// THEN: it still passes, but no longer passes the strict checks
	EXPECT_TRUE(_checks.passed());
	EXPECT_FALSE(_checks.strict());
	EXPECT_FALSE(_checks.passedStrict());
}

TEST_F(GnssChecksTest, strictFixType)
{
	// GIVEN: a DGPS fix required, and a receiver that passed the strict checks, then took off
	GnssChecks::Params params{};
	params.req_fix = 4;
	setParams(params);
	_sample.fix_type = 4;

	runSeconds(20.f, false, false);
	runSeconds(1.f, true, true);

	EXPECT_TRUE(_checks.passedStrict());

	// WHEN: it drops to a 3D fix, which the relaxed checks accept
	_sample.fix_type = 3;
	runSeconds(1.f, true, true);

	// THEN: it no longer passes the strict checks
	EXPECT_TRUE(_checks.passed());
	EXPECT_FALSE(_checks.passedStrict());
}

TEST_F(GnssChecksTest, strictFollowsCheckMask)
{
	// GIVEN: the horizontal accuracy check disabled
	GnssChecks::Params params{};
	params.check_mask &= ~vehicle_gnss_s::CHECK_EPH;
	setParams(params);

	// WHEN: the horizontal accuracy exceeds its threshold
	_sample.hacc = 10.f;
	runSeconds(15.f, false, false);

	// THEN: it passes the strict checks
	EXPECT_TRUE(_checks.passedStrict());

	// WHEN: the speed accuracy exceeds its enabled threshold
	_sample.sacc = 1.f;
	runSeconds(1.f, false, false);

	// THEN: they aren't
	EXPECT_FALSE(_checks.passedStrict());
}

class GnssChecksPosVelTest : public GnssChecksTest
{
public:
	void SetUp() override
	{
		GnssChecksTest::SetUp();

		// GIVEN: the default checks, which include the position/velocity consistency, and a receiver that passed them
		runSeconds(2.f, false, false);
	}
};

TEST_F(GnssChecksPosVelTest, failsWhenPositionWandersFromVelocity)
{
	// WHEN: in flight, the position moves as the velocity says
	flySeconds(2.f, matrix::Vector3f(8.f, -6.f, 0.f));

	// THEN: it passes the strict checks
	EXPECT_FALSE(_checks.getStrictFailFlags() & vehicle_gnss_s::CHECK_POSVEL);
	EXPECT_TRUE(_checks.passedStrict());

	// WHEN: the position walks off at 3 m/s that the velocity doesn't show, as with multipath
	flySeconds(5.f, matrix::Vector3f(8.f, -6.f, 0.f), matrix::Vector3f(0.f, 3.f, 0.f));

	// THEN: it fails the strict checks on the consistency, while the relaxed checks still let it be used
	EXPECT_TRUE(_checks.getStrictFailFlags() & vehicle_gnss_s::CHECK_POSVEL);
	EXPECT_FALSE(_checks.passedStrict());
	EXPECT_TRUE(_checks.passed());

	// WHEN: the position follows the velocity again
	flySeconds(5.f, matrix::Vector3f(8.f, -6.f, 0.f));

	// THEN: it passes the strict checks again
	EXPECT_FALSE(_checks.getStrictFailFlags() & vehicle_gnss_s::CHECK_POSVEL);
	EXPECT_TRUE(_checks.passedStrict());
}

TEST_F(GnssChecksPosVelTest, skipsPositionStepAtFixTypeChange)
{
	// WHEN: in flight, the position steps by 3 m in the epoch the receiver starts applying corrections, as it does
	// when a differential or RTK solution starts or stops
	flySeconds(2.f, matrix::Vector3f(8.f, -6.f, 0.f));
	_pos_ned(0) += 3.f;
	_sample.fix_type = 4;
	flySeconds(0.1f, matrix::Vector3f(8.f, -6.f, 0.f));

	// THEN: the step doesn't count, and it passes the consistency check
	EXPECT_FALSE(_checks.getStrictFailFlags() & vehicle_gnss_s::CHECK_POSVEL);

	// WHEN: the position then walks off at 3 m/s that the velocity doesn't show
	flySeconds(5.f, matrix::Vector3f(8.f, -6.f, 0.f), matrix::Vector3f(0.f, 3.f, 0.f));

	// THEN: it still fails it
	EXPECT_TRUE(_checks.getStrictFailFlags() & vehicle_gnss_s::CHECK_POSVEL);
}
