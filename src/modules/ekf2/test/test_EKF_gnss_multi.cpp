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
 * Test the fusion of two GNSS receivers
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfGnssMultiTest : public ::testing::Test
{
public:
	EkfGnssMultiTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion(0);
		_ekf_wrapper.enableGpsFusion(1);
		_sensor_simulator.startGps();
		_sensor_simulator.startGps1();
		_sensor_simulator.runSeconds(11);

		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);
		_sensor_simulator.runSeconds(2);
	}

	bool fused(uint8_t slot) const
	{
		return _ekf->aid_src_gnss_pos(slot).fused && _ekf->aid_src_gnss_vel(slot).fused;
	}
};

TEST_F(EkfGnssMultiTest, bothReceiversFused)
{
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(fused(0));
	EXPECT_TRUE(fused(1));
}

TEST_F(EkfGnssMultiTest, inconsistentReceiverDoesNotReset)
{
	// GIVEN: both receivers fused
	ASSERT_TRUE(fused(0));
	ASSERT_TRUE(fused(1));

	const uint8_t pos_reset_count = _ekf->get_posNE_reset_count();
	const uint8_t vel_reset_count = _ekf->get_velNE_reset_count();

	// WHEN: the second receiver jumps away from the true position
	_sensor_simulator._gps1.stepHorizontalPositionByMeters(Vector2f(20.f, 0.f));
	_sensor_simulator.runSeconds(15);

	// THEN: the second receiver is rejected and never resets the state while the first one keeps fusing
	EXPECT_EQ(_ekf->get_posNE_reset_count(), pos_reset_count);
	EXPECT_EQ(_ekf->get_velNE_reset_count(), vel_reset_count);
	EXPECT_LT(Vector2f(_ekf->getPosition()).norm(), 1.f);

	EXPECT_TRUE(fused(0));
	EXPECT_FALSE(_ekf->aid_src_gnss_pos(1).fused);
}

TEST_F(EkfGnssMultiTest, disablingOneReceiverKeepsTheOther)
{
	// GIVEN: both receivers fused
	ASSERT_TRUE(fused(0));
	ASSERT_TRUE(fused(1));

	const uint8_t pos_reset_count = _ekf->get_posNE_reset_count();

	// WHEN: the first receiver is disabled at runtime
	_ekf_wrapper.setGpsEnabled(false, 0);
	_sensor_simulator.runSeconds(5);

	// THEN: the second receiver keeps constraining the position without a reset
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_FALSE(_ekf->gnssSource(0).isActive(_ekf->control_status()));
	EXPECT_TRUE(fused(1));
	EXPECT_EQ(_ekf->get_posNE_reset_count(), pos_reset_count);
}
