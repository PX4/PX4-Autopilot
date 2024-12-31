/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * Test EKF altitude estimation while grounded and using Rangefinder as primary alt source
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfGroundedTest : public ::testing::Test
{
public:

	EkfGroundedTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void SetUp() override
	{
		// Init EKF
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator._rng.setData(0.1f, 100);
		_sensor_simulator._rng.setLimits(0.1f, 25.f);

		_sensor_simulator.startGps();
		_sensor_simulator.startBaro();
		_sensor_simulator.startRangeFinder();

		// Set Range as primary height source
		_ekf_wrapper.setRangeHeightRef();

		// Enable fusion for all height sources
		_ekf_wrapper.enableBaroHeightFusion();
		_ekf_wrapper.enableGpsHeightFusion();
		_ekf_wrapper.enableRangeHeightFusion();

		// Give EKF time for GPS
		_sensor_simulator.runSeconds(20);
	}
};

TEST_F(EkfGroundedTest, rangeFinderOnGround)
{
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::RANGE);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());

	float distance = 0.17f;
	_sensor_simulator._rng.setData(distance, 100);

	_sensor_simulator.runSeconds(60);

	EXPECT_NEAR(_ekf->getLatLonAlt().altitude(), 0, 0.5f);
}
