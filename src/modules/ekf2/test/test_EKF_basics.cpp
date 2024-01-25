/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include <chrono>
#include <gtest/gtest.h>
#include <math.h>
#include <memory>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfBasicsTest : public ::testing::Test
{
public:
	EkfBasicsTest():
		::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_ekf_wrapper(_ekf),
		_sensor_simulator(_ekf)
	{
	};

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(_init_duration_s);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	std::shared_ptr<Ekf> _ekf {nullptr};
	EkfWrapper _ekf_wrapper;
	SensorSimulator _sensor_simulator;

	// Duration of initalization with only providing baro,mag and IMU
	const uint32_t _init_duration_s{5};

protected:
	double _latitude  {0.0};
	double _longitude {0.0};
	float  _altitude  {0.f};

	double _latitude_new  {0.0};
	double _longitude_new {0.0};
	float  _altitude_new  {0.f};

	uint64_t _origin_time = 0;

private:

};


TEST_F(EkfBasicsTest, tiltAlign)
{
	// GIVEN: reasonable static sensor data for some duration
	// THEN: EKF should tilt align
	EXPECT_TRUE(_ekf->attitude_valid());
}

TEST_F(EkfBasicsTest, initialControlMode)
{
	// GIVEN: reasonable static sensor data for some duration
	// THEN: EKF control status should be reasonable
	EXPECT_EQ(1, (int) _ekf->control_status_flags().tilt_align);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gps);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().opt_flow);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().mag_hdg);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_3D);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_dec);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().in_air);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().wind);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().baro_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().rng_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gps_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_pos);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_yaw);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fuse_beta);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_field_disturbed);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fixed_wing);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_fault);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gnd_effect);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().rng_stuck);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gps_yaw);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_aligned_in_flight);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_vel);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().synthetic_mag_z);
}

TEST_F(EkfBasicsTest, convergesToZero)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input
	_sensor_simulator.runSeconds(4);

	const Vector3f pos = _ekf->getPosition();
	const Vector3f vel = _ekf->getVelocity();
	const Vector3f accel_bias = _ekf->getAccelBias();
	const Vector3f gyro_bias = _ekf->getGyroBias();
	const Vector3f ref{0.0f, 0.0f, 0.0f};

	// THEN: EKF should stay or converge to zero
	EXPECT_TRUE(matrix::isEqual(pos, ref, 0.001f));
	EXPECT_TRUE(matrix::isEqual(vel, ref, 0.001f));
	EXPECT_TRUE(matrix::isEqual(accel_bias, ref, 0.001f));
	EXPECT_TRUE(matrix::isEqual(gyro_bias, ref, 0.001f));
}

TEST_F(EkfBasicsTest, gpsFusion)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input for
	// WHEN: setting GPS measurements for 11s, minimum GPS health time is set to 10 sec

	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should fuse GPS, but no other position sensor
	EXPECT_EQ(1, (int) _ekf->control_status_flags().tilt_align);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().gps);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().opt_flow);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().mag_hdg);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_3D);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_dec);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().in_air);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().wind);
	EXPECT_EQ(1, (int) _ekf->control_status_flags().baro_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().rng_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gps_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_pos);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_yaw);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_hgt);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fuse_beta);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_field_disturbed);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().fixed_wing);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_fault);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gnd_effect);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().rng_stuck);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().gps_yaw);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().mag_aligned_in_flight);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().ev_vel);
	EXPECT_EQ(0, (int) _ekf->control_status_flags().synthetic_mag_z);
}

TEST_F(EkfBasicsTest, accelBiasEstimation)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input
	// WHEN: Added more sensor measurements with accel bias and gps measurements
	const Vector3f accel_bias_sim = {0.0f, 0.0f, 0.1f};

	_sensor_simulator.startGps();
	_sensor_simulator.setImuBias(accel_bias_sim, Vector3f(0.0f, 0.0f, 0.0f));
	_ekf->set_min_required_gps_health_time(1e6);
	_sensor_simulator.runSeconds(60);

	const Vector3f pos = _ekf->getPosition();
	const Vector3f vel = _ekf->getVelocity();
	const Vector3f accel_bias = _ekf->getAccelBias();
	const Vector3f gyro_bias = _ekf->getGyroBias();
	const Vector3f zero = {0.0f, 0.0f, 0.0f};

	// THEN: EKF should stay or converge to zero
	EXPECT_TRUE(matrix::isEqual(pos, zero, 0.05f))
			<< "pos = " << pos(0) << ", " << pos(1) << ", " << pos(2);
	EXPECT_TRUE(matrix::isEqual(vel, zero, 0.02f))
			<< "vel = " << vel(0) << ", " << vel(1) << ", " << vel(2);
	EXPECT_TRUE(matrix::isEqual(accel_bias, accel_bias_sim, 0.01f))
			<< "accel_bias = " << accel_bias(0) << ", " << accel_bias(1) << ", " << accel_bias(2);
	EXPECT_TRUE(matrix::isEqual(gyro_bias, zero, 0.001f))
			<< "gyro_bias = " << gyro_bias(0) << ", " << gyro_bias(1) << ", " << gyro_bias(2);
}

TEST_F(EkfBasicsTest, reset_ekf_global_origin_gps_initialized)
{
	_latitude_new  = 15.0000005;
	_longitude_new = 115.0000005;
	_altitude_new  = 100.0;

	_sensor_simulator.startGps();
	_ekf->set_min_required_gps_health_time(1e6);
	_sensor_simulator.runSeconds(1);

	_sensor_simulator.setGpsLatitude(_latitude_new);
	_sensor_simulator.setGpsLongitude(_longitude_new);
	_sensor_simulator.setGpsAltitude(_altitude_new);
	_sensor_simulator.runSeconds(5);

	_ekf->getEkfGlobalOrigin(_origin_time, _latitude, _longitude, _altitude);

	EXPECT_DOUBLE_EQ(_latitude, _latitude_new);
	EXPECT_DOUBLE_EQ(_longitude, _longitude_new);
	EXPECT_NEAR(_altitude, _altitude_new, 0.01f);

	// Note: we cannot reset too far since the local position is limited to 1e6m
	_latitude_new  = 14.0000005;
	_longitude_new = 109.0000005;
	_altitude_new  = 1500.0;

	_ekf->setEkfGlobalOrigin(_latitude_new, _longitude_new, _altitude_new);
	_ekf->getEkfGlobalOrigin(_origin_time, _latitude, _longitude, _altitude);

	EXPECT_DOUBLE_EQ(_latitude, _latitude_new);
	EXPECT_DOUBLE_EQ(_longitude, _longitude_new);
	EXPECT_FLOAT_EQ(_altitude, _altitude_new);

	_sensor_simulator.runSeconds(1);

	// After the change of origin, the pos and vel innovations should stay small
	EXPECT_NEAR(_ekf->aid_src_gnss_pos().test_ratio[0], 0.f, 0.05f);
	EXPECT_NEAR(_ekf->aid_src_gnss_pos().test_ratio[1], 0.f, 0.05f);
	EXPECT_NEAR(_ekf->aid_src_gnss_hgt().test_ratio, 0.f, 0.05f);

	EXPECT_NEAR(_ekf->aid_src_baro_hgt().test_ratio, 0.f, 0.05f);

	EXPECT_NEAR(_ekf->aid_src_gnss_vel().test_ratio[0], 0.f, 0.02f);
	EXPECT_NEAR(_ekf->aid_src_gnss_vel().test_ratio[1], 0.f, 0.02f);
	EXPECT_NEAR(_ekf->aid_src_gnss_vel().test_ratio[2], 0.f, 0.02f);
}

TEST_F(EkfBasicsTest, reset_ekf_global_origin_gps_uninitialized)
{
	_ekf->getEkfGlobalOrigin(_origin_time, _latitude_new, _longitude_new, _altitude_new);

	EXPECT_DOUBLE_EQ(_latitude, _latitude_new);
	EXPECT_DOUBLE_EQ(_longitude, _longitude_new);
	EXPECT_FLOAT_EQ(_altitude, _altitude_new);

	EXPECT_FALSE(_ekf->global_origin_valid());

	_latitude_new  = 45.0000005;
	_longitude_new = 111.0000005;
	_altitude_new  = 1500.0;

	_ekf->setEkfGlobalOrigin(_latitude_new, _longitude_new, _altitude_new);
	_ekf->getEkfGlobalOrigin(_origin_time, _latitude, _longitude, _altitude);

	EXPECT_DOUBLE_EQ(_latitude, _latitude_new);
	EXPECT_DOUBLE_EQ(_longitude, _longitude_new);
	EXPECT_FLOAT_EQ(_altitude, _altitude_new);

	// Global origin has been initialized but since there is no position aiding, the global
	// position is still not valid
	EXPECT_TRUE(_ekf->global_origin_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	_sensor_simulator.runSeconds(1);

	// After the change of origin, the pos and vel innovations should stay small
	EXPECT_NEAR(_ekf->aid_src_gnss_pos().test_ratio[0], 0.f, 0.05f);
	EXPECT_NEAR(_ekf->aid_src_gnss_pos().test_ratio[1], 0.f, 0.05f);
	EXPECT_NEAR(_ekf->aid_src_gnss_hgt().test_ratio, 0.f, 0.05f);

	EXPECT_NEAR(_ekf->aid_src_gnss_vel().test_ratio[0], 0.f, 0.02f);
	EXPECT_NEAR(_ekf->aid_src_gnss_vel().test_ratio[1], 0.f, 0.02f);
	EXPECT_NEAR(_ekf->aid_src_gnss_vel().test_ratio[2], 0.f, 0.02f);
}

TEST_F(EkfBasicsTest, global_position_from_local_ev)
{
	// GIVEN: external vision (local) aiding
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator._vio.setPositionFrameToLocalNED();
	_sensor_simulator.startExternalVision();

	_sensor_simulator.runSeconds(1);

	// THEN; since there is no origin, only the local position can be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_origin_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	_latitude_new  = 45.0000005;
	_longitude_new = 111.0000005;
	_altitude_new  = 1500.0;

	// BUT WHEN: the global origin is set (manually)
	_ekf->setEkfGlobalOrigin(_latitude_new, _longitude_new, _altitude_new);

	// THEN: local and global positions are valid
	EXPECT_TRUE(_ekf->global_origin_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());
	EXPECT_TRUE(_ekf->local_position_is_valid());
}

TEST_F(EkfBasicsTest, global_position_from_opt_flow)
{
	// GIVEN: optical flow aiding
	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator.startFlow();
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.startRangeFinder();

	_sensor_simulator.runSeconds(1);

	// THEN; since there is no origin, only the local position can be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_origin_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	_latitude_new  = 45.0000005;
	_longitude_new = 111.0000005;
	_altitude_new  = 1500.0;

	// BUT WHEN: the global origin is set (manually)
	_ekf->setEkfGlobalOrigin(_latitude_new, _longitude_new, _altitude_new);

	// THEN: local and global positions are valid
	EXPECT_TRUE(_ekf->global_origin_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());
	EXPECT_TRUE(_ekf->local_position_is_valid());
}

// TODO: Add sampling tests
