/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <lib/geo/geo.h>

#include "AirspeedValidator.hpp"

using namespace time_literals;

class AirspeedValidatorLoadFactorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		_validator.set_airspeed_stall(kStallAirspeed);
		_validator.set_checks_fail_delay(kChecksFailDelay);
		_validator.set_enable_load_factor_check(true);

		// run on ground first such that the checks have passed once before the flight starts
		for (int i = 0; i < 10; i++) {
			update(0.f, 1.f, false);
		}

		ASSERT_TRUE(_validator.get_airspeed_valid());
	}

	void update(float indicated_airspeed, float load_factor, bool in_fixed_wing_flight)
	{
		_timestamp += kSampleInterval;

		airspeed_validator_update_data input_data{};
		input_data.timestamp = _timestamp;
		input_data.airspeed_indicated_raw = indicated_airspeed;
		input_data.airspeed_true_raw = indicated_airspeed;
		input_data.airspeed_timestamp = _timestamp;
		input_data.q_att = matrix::Quatf();
		input_data.air_pressure_pa = 101325.f;
		input_data.air_temperature_celsius = 15.f;
		input_data.accel_z = -load_factor * CONSTANTS_ONE_G;
		input_data.in_fixed_wing_flight = in_fixed_wing_flight;

		_validator.update_airspeed_validator(input_data);
	}

	static constexpr float kStallAirspeed = 17.f;
	static constexpr float kCruiseAirspeed = 20.f;
	static constexpr float kChecksFailDelay = 1.f;
	static constexpr uint64_t kSampleInterval = 100_ms;

	AirspeedValidator _validator;
	uint64_t _timestamp{1_s};
};

TEST_F(AirspeedValidatorLoadFactorTest, singleLowAirspeedSampleAfterLaunch)
{
	// GIVEN: the check starts right after launch, while the airspeed reading has not caught up yet
	update(2.4f, 1.7f, true);

	// WHEN: all following samples are at cruise airspeed in level flight
	for (int i = 0; i < 50; i++) {
		update(kCruiseAirspeed, 1.f, true);

		// THEN: the single sample does not invalidate the airspeed
		EXPECT_TRUE(_validator.get_airspeed_valid()) << "sample " << i;
	}
}

TEST_F(AirspeedValidatorLoadFactorTest, blockedSensorInCruise)
{
	// GIVEN: level cruise flight with a healthy sensor
	for (int i = 0; i < 50; i++) {
		update(kCruiseAirspeed, 1.f, true);
	}

	ASSERT_TRUE(_validator.get_airspeed_valid());

	// WHEN: the airspeed reading drops to zero
	const uint64_t time_blocked = _timestamp;

	while (_validator.get_airspeed_valid() && (_timestamp - time_blocked) < 5_s) {
		update(0.f, 1.f, true);
	}

	// THEN: the airspeed is invalidated shortly after the fail delay
	EXPECT_FALSE(_validator.get_airspeed_valid());
	EXPECT_LE(_timestamp - time_blocked, static_cast<uint64_t>(kChecksFailDelay * 1_s) + 500_ms);
}
