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

/**
 * @file test_EKF_gravity_fusion.cpp
 *
 * Regression for https://github.com/PX4/PX4-Autopilot/issues/24299
 *
 * Without horizontal aiding, tilt is constrained only by gravity fusion. Near
 * the ground, throttle hunting and residual rates can build tilt error; gravity
 * fusion must re-acquire level when returning to hover (mid-stick).
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfGravityFusionTest : public ::testing::Test
{
public:
	EkfGravityFusionTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		// No mag / GPS: tilt only from gravity fusion (the #24299 case).
		_ekf_wrapper.setMagFuseTypeNone();
		_ekf_wrapper.enableGravityFusion();

		_sensor_simulator.simulateOrientation(Quatf());
		_sensor_simulator.runSeconds(7);

		ASSERT_TRUE(_ekf->attitude_valid());
		ASSERT_TRUE(_ekf->control_status_flags().tilt_align);
	}
};

// Reproduce #24299: residual rate while |a| is elevated (near-ground throttle /
// manoeuvre) builds tilt error; returning to hover (mid-stick) must recover level.
TEST_F(EkfGravityFusionTest, recoversLevelAfterAccelGateDropout)
{
	// GIVEN: in air, no horizontal aiding, gravity fusion enabled
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_in_air_status(true);
	_sensor_simulator.runSeconds(1.f);

	// Steady hover: gravity fusion should be active
	_sensor_simulator._imu.setData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G), Vector3f());
	_sensor_simulator.runSeconds(1.f);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGravityFusion());

	const Eulerf euler_before = _ekf_wrapper.getEulerAngles();
	EXPECT_NEAR(euler_before.phi(), 0.f, math::radians(3.f));
	EXPECT_NEAR(euler_before.theta(), 0.f, math::radians(3.f));

	// WHEN: elevated |a| (still within soft enable band) + residual body rate so
	// tilt error grows (weakened fusion while manoeuvring / throttle hunting).
	const float rate_rad_s = math::radians(8.f); // ~8 deg/s
	const float manoeuvre_s = 4.f;
	_sensor_simulator._imu.setData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G * 1.25f),
				       Vector3f(rate_rad_s, 0.f, 0.f));
	_sensor_simulator.runSeconds(manoeuvre_s);

	// Soft enable keeps fusion intended at 1.25 g (hard window used to drop out).
	EXPECT_TRUE(_ekf_wrapper.isIntendingGravityFusion());

	const Eulerf euler_drifted = _ekf_wrapper.getEulerAngles();
	// Inflated measurement noise during manoeuvre still allows some tilt build-up
	EXPECT_GT(fabsf(euler_drifted.phi()), math::radians(10.f));

	// AND WHEN: back to steady hover (|a| ≈ g, zero rate) — mid-stick / level case
	_sensor_simulator._imu.setData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G), Vector3f());
	_sensor_simulator.runSeconds(8.f);

	// THEN: gravity fusion pulls attitude back to level (requires innovation gate
	// wide enough to accept a large residual after manoeuvre).
	EXPECT_TRUE(_ekf_wrapper.isIntendingGravityFusion());

	const Eulerf euler_after = _ekf_wrapper.getEulerAngles();
	EXPECT_NEAR(euler_after.phi(), 0.f, math::radians(5.f))
			<< "roll after recovery: " << math::degrees(euler_after.phi()) << " deg";
	EXPECT_NEAR(euler_after.theta(), 0.f, math::radians(5.f))
			<< "pitch after recovery: " << math::degrees(euler_after.theta()) << " deg";
}

// Sanity: gravity fusion stays enabled in steady hover ( |a| ≈ g ).
TEST_F(EkfGravityFusionTest, gravityFusionActiveInSteadyHover)
{
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_in_air_status(true);

	_sensor_simulator._imu.setData(Vector3f(0.f, 0.f, -CONSTANTS_ONE_G), Vector3f());
	_sensor_simulator.runSeconds(3.f);

	EXPECT_TRUE(_ekf_wrapper.isIntendingGravityFusion());

	const Eulerf euler = _ekf_wrapper.getEulerAngles();
	EXPECT_NEAR(euler.phi(), 0.f, math::radians(3.f));
	EXPECT_NEAR(euler.theta(), 0.f, math::radians(3.f));
}
