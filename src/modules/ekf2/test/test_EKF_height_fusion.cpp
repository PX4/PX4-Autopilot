/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * Height fusion logic
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfHeightFusionTest : public ::testing::Test
{
public:

	EkfHeightFusionTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		_ekf->init(0);
		_ekf_wrapper.disableBaroHeightFusion();
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.startGps();

		_sensor_simulator._rng.setData(1.f, 100);
		_sensor_simulator._rng.setLimits(0.1f, 9.f);
		_sensor_simulator.startRangeFinder();

		_sensor_simulator.startExternalVision();

		_sensor_simulator.runSeconds(15);

		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EkfHeightFusionTest, noAiding)
{
	EXPECT_FALSE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeightFusion());
}

TEST_F(EkfHeightFusionTest, baroRef)
{
	// GIVEN: baro reference with GPS and range height fusion
	_ekf_wrapper.setBaroHeightRef();
	_ekf_wrapper.enableBaroHeightFusion();
	_sensor_simulator.runSeconds(0.1);

	_ekf_wrapper.enableGpsHeightFusion();
	_ekf_wrapper.enableRangeHeightFusion();
	/* _ekf_wrapper.enableExternalVisionHeightFusion(); */ //TODO: this currently sets the reference to EV
	_sensor_simulator.runSeconds(1);

	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::BARO);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeightFusion());

	// AND WHEN: the baro data increases
	const float baro_increment = 5.f;
	_sensor_simulator._baro.setData(_sensor_simulator._baro.getData() + baro_increment);
	_sensor_simulator.runSeconds(60);

	// THEN: the height estimate converges to the baro value
	// and the other height sources are getting their bias estimated
	EXPECT_NEAR(_ekf->getPosition()(2), -baro_increment, 0.1f);
	const BiasEstimator::status &baro_status = _ekf->getBaroBiasEstimatorStatus();
	/* EXPECT_EQ(status.bias, _sensor_simulator._baro.getData()); */ // This is the real bias, but the estimator isn't running so the status isn't updated
	EXPECT_EQ(baro_status.bias, 0.f);
	const BiasEstimator::status &gps_status = _ekf->getGpsHgtBiasEstimatorStatus();
	EXPECT_NEAR(gps_status.bias, -baro_increment, 0.2f);

	const BiasEstimator::status &rng_status = _ekf->getRngHgtBiasEstimatorStatus();
	EXPECT_NEAR(rng_status.bias, -baro_increment, 1.2f);

	const BiasEstimator::status &ev_status = _ekf->getEvHgtBiasEstimatorStatus();
	EXPECT_EQ(ev_status.bias, 0.f);

	// BUT WHEN: the baro data jumps by a lot
	_sensor_simulator._baro.setData(_sensor_simulator._baro.getData() + 200.f);
	_sensor_simulator.runSeconds(10);

	// THEN: the baro is stopped and the GPS takes the role of the height reference
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::GNSS);
	EXPECT_FALSE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());

	// AND WHEN: the gps height increases
	const float gps_increment = 1.f;
	_sensor_simulator._gps.stepHeightByMeters(gps_increment);
	_sensor_simulator.runSeconds(60);

	// THEN: the GPS bias stays constant
	EXPECT_EQ(gps_status.bias, _ekf->getGpsHgtBiasEstimatorStatus().bias);
	// the estimated height follows the GPS height
	EXPECT_NEAR(_ekf->getPosition()(2), -(baro_increment + gps_increment), 0.3f);
	// and the range finder bias is adjusted to follow the new reference
	const BiasEstimator::status &rng_status_2 = _ekf->getRngHgtBiasEstimatorStatus();
	EXPECT_NEAR(rng_status_2.bias, -(baro_increment + gps_increment), 1.3f);
}

TEST_F(EkfHeightFusionTest, gpsRef)
{
	// GIVEN: GPS reference, baro and range height fusion
	_ekf_wrapper.setGpsHeightRef();
	_ekf_wrapper.enableBaroHeightFusion();
	_ekf_wrapper.enableGpsHeightFusion();
	_ekf_wrapper.enableRangeHeightFusion();
	_sensor_simulator.runSeconds(1);

	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::GNSS);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeightFusion());

	// AND WHEN: the baro data increases
	const float baro_initial = _sensor_simulator._baro.getData();
	const float baro_increment = 5.f;
	_sensor_simulator._baro.setData(baro_initial + baro_increment);
	_sensor_simulator.runSeconds(100);

	// THEN: the height estimate is temporarily biased but then converges back to
	// the GPS height value and the baro gets its bias estimated
	EXPECT_NEAR(_ekf->getPosition()(2), 0.f, 1.f);
	const BiasEstimator::status &baro_status = _ekf->getBaroBiasEstimatorStatus();
	EXPECT_NEAR(baro_status.bias, baro_initial + baro_increment, 1.3f);

	const BiasEstimator::status &rng_status = _ekf->getRngHgtBiasEstimatorStatus();
	EXPECT_NEAR(rng_status.bias, 0.f, 1.1f); // TODO: why?

	// BUT WHEN: the GPS jumps by a lot
	const float gps_step = 100.f;
	_sensor_simulator._gps.stepHeightByMeters(gps_step);
	_sensor_simulator.runSeconds(10);

	// THEN: the height is reset to the new GPS altitude and all the bias estimates are updated accordingly
	EXPECT_NEAR(_ekf->getPosition()(2), -gps_step, 1.f);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::GNSS);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_NEAR(_ekf->getBaroBiasEstimatorStatus().bias, baro_initial + baro_increment - gps_step, 0.2f);

	// and the innovations are close to zero
	EXPECT_NEAR(_ekf->aid_src_baro_hgt().innovation, 0.f, 0.2f);
	EXPECT_NEAR(_ekf->aid_src_rng_hgt().innovation, 0.f, 0.2f);
}

TEST_F(EkfHeightFusionTest, baroRefFailOver)
{
	// GIVEN: baro reference with GPS and range height fusion
	_ekf_wrapper.setBaroHeightRef();
	_ekf_wrapper.enableBaroHeightFusion();
	_ekf_wrapper.enableGpsHeightFusion();
	_ekf_wrapper.enableRangeHeightFusion();

	_sensor_simulator.runSeconds(0.1);

	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::BARO);

	_sensor_simulator.stopBaro();
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::GNSS);

	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::RANGE);

	_sensor_simulator.stopRangeFinder();
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::UNKNOWN);
}

TEST_F(EkfHeightFusionTest, gpsRefFailOver)
{
	// GIVEN: baro reference with GPS and range height fusion
	_sensor_simulator.startBaro();
	_sensor_simulator.startGps();
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.setGpsHeightRef();
	_ekf_wrapper.enableBaroHeightFusion();
	_ekf_wrapper.enableGpsHeightFusion();
	_ekf_wrapper.enableRangeHeightFusion();

	// The GPS takes time to start, use baro at first
	_sensor_simulator.runSeconds(0.1);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::BARO);

	// Then switch to reference height source
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::GNSS);

	// And switch to the fallback sources once the current reference fails
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::BARO);

	_sensor_simulator.stopBaro();
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::RANGE);

	_sensor_simulator.stopRangeFinder();
	_sensor_simulator.runSeconds(10);
	EXPECT_TRUE(_ekf->getHeightSensorRef() == HeightSensor::UNKNOWN);
}

TEST_F(EkfHeightFusionTest, changeEkfOriginAlt)
{
	_sensor_simulator.startBaro();
	_sensor_simulator.startGps();
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.setGpsHeightRef();
	_ekf_wrapper.enableBaroHeightFusion();
	_ekf_wrapper.enableGpsHeightFusion();
	_ekf_wrapper.enableRangeHeightFusion();
	_sensor_simulator.runSeconds(10);

	uint64_t origin_time;
	double lat;
	double lon;
	float alt;
	_ekf->getEkfGlobalOrigin(origin_time, lat, lon, alt);

	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();

	const float alt_increment = 4478.f;
	_ekf->setEkfGlobalOrigin(lat, lon, alt + alt_increment);
	_sensor_simulator.runSeconds(10);

	// The origin moves up by some altitude, the current position (down) is then higher
	EXPECT_NEAR(_ekf->getPosition()(2), alt_increment, 1.f);

	reset_logging_checker.capturePostResetState();
	EXPECT_NEAR(_ekf->getBaroBiasEstimatorStatus().bias, _sensor_simulator._baro.getData() + alt_increment, 0.2f);

	EXPECT_NEAR(_ekf->getRngHgtBiasEstimatorStatus().bias, alt_increment, 1.f);
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));
}
