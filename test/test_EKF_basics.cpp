/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
#include <math.h>
#include <memory>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"


class EkfInitializationTest : public ::testing::Test {
 public:
	EkfInitializationTest(): ::testing::Test(),
	_ekf{std::make_shared<Ekf>()},
	_sensor_simulator(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;

	// Duration of initalization with only providing baro,mag and IMU
	const uint32_t _init_duration_s{2};

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.run_seconds(_init_duration_s);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};


TEST_F(EkfInitializationTest, tiltAlign)
{
	// GIVEN: reasonable static sensor data for some duration
	// THEN: EKF should tilt align
	EXPECT_EQ(true,_ekf->attitude_valid());
}

TEST_F(EkfInitializationTest, initialControlMode)
{
	// GIVEN: reasonable static sensor data for some duration
	// THEN: EKF control status should be reasonable
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);

	EXPECT_EQ(1, (int) control_status.flags.tilt_align);
	EXPECT_EQ(1, (int) control_status.flags.yaw_align);
	EXPECT_EQ(0, (int) control_status.flags.gps);
	EXPECT_EQ(0, (int) control_status.flags.opt_flow);
	EXPECT_EQ(1, (int) control_status.flags.mag_hdg);
	EXPECT_EQ(0, (int) control_status.flags.mag_3D);
	EXPECT_EQ(0, (int) control_status.flags.mag_dec);
	EXPECT_EQ(0, (int) control_status.flags.in_air);
	EXPECT_EQ(0, (int) control_status.flags.wind);
	EXPECT_EQ(1, (int) control_status.flags.baro_hgt);
	EXPECT_EQ(0, (int) control_status.flags.rng_hgt);
	EXPECT_EQ(0, (int) control_status.flags.gps_hgt);
	EXPECT_EQ(0, (int) control_status.flags.ev_pos);
	EXPECT_EQ(0, (int) control_status.flags.ev_yaw);
	EXPECT_EQ(0, (int) control_status.flags.ev_hgt);
	EXPECT_EQ(0, (int) control_status.flags.fuse_beta);
	EXPECT_EQ(0, (int) control_status.flags.mag_field_disturbed);
	EXPECT_EQ(0, (int) control_status.flags.fixed_wing);
	EXPECT_EQ(0, (int) control_status.flags.mag_fault);
	EXPECT_EQ(0, (int) control_status.flags.gnd_effect);
	EXPECT_EQ(0, (int) control_status.flags.rng_stuck);
	EXPECT_EQ(0, (int) control_status.flags.gps_yaw);
	EXPECT_EQ(0, (int) control_status.flags.mag_aligned_in_flight);
	EXPECT_EQ(0, (int) control_status.flags.ev_vel);
	EXPECT_EQ(0, (int) control_status.flags.synthetic_mag_z);
}

TEST_F(EkfInitializationTest, convergesToZero)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input
	_sensor_simulator.run_seconds(4);

	float converged_pos[3];
	float converged_vel[3];
	float converged_accel_bias[3];
	float converged_gyro_bias[3];
	_ekf->get_position(converged_pos);
	_ekf->get_velocity(converged_vel);
	_ekf->get_accel_bias(converged_accel_bias);
	_ekf->get_gyro_bias(converged_gyro_bias);

	// THEN: EKF should stay or converge to zero
	for(int i=0; i<3; ++i)
	{
		EXPECT_NEAR(0.0f,converged_pos[i],0.001f);
		EXPECT_NEAR(0.0f,converged_vel[i],0.001f);
		EXPECT_NEAR(0.0f,converged_accel_bias[i],0.001f);
		EXPECT_NEAR(0.0f,converged_gyro_bias[i],0.001f);
	}
}

TEST_F(EkfInitializationTest, gpsFusion)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input for
	// WHEN: setting GPS measurements for 11s, minimum GPS health time is set to 10 sec

	_sensor_simulator.startGps();
	_sensor_simulator.run_seconds(11);

	// THEN: EKF should fuse GPS, but no other position sensor
	filter_control_status_u control_status;
	_ekf->get_control_mode(&control_status.value);
	EXPECT_EQ(1, (int) control_status.flags.tilt_align);
	EXPECT_EQ(1, (int) control_status.flags.yaw_align);
	EXPECT_EQ(1, (int) control_status.flags.gps);
	EXPECT_EQ(0, (int) control_status.flags.opt_flow);
	EXPECT_EQ(1, (int) control_status.flags.mag_hdg);
	EXPECT_EQ(0, (int) control_status.flags.mag_3D);
	EXPECT_EQ(0, (int) control_status.flags.mag_dec);
	EXPECT_EQ(0, (int) control_status.flags.in_air);
	EXPECT_EQ(0, (int) control_status.flags.wind);
	EXPECT_EQ(1, (int) control_status.flags.baro_hgt);
	EXPECT_EQ(0, (int) control_status.flags.rng_hgt);
	EXPECT_EQ(0, (int) control_status.flags.gps_hgt);
	EXPECT_EQ(0, (int) control_status.flags.ev_pos);
	EXPECT_EQ(0, (int) control_status.flags.ev_yaw);
	EXPECT_EQ(0, (int) control_status.flags.ev_hgt);
	EXPECT_EQ(0, (int) control_status.flags.fuse_beta);
	EXPECT_EQ(0, (int) control_status.flags.mag_field_disturbed);
	EXPECT_EQ(0, (int) control_status.flags.fixed_wing);
	EXPECT_EQ(0, (int) control_status.flags.mag_fault);
	EXPECT_EQ(0, (int) control_status.flags.gnd_effect);
	EXPECT_EQ(0, (int) control_status.flags.rng_stuck);
	EXPECT_EQ(0, (int) control_status.flags.gps_yaw);
	EXPECT_EQ(0, (int) control_status.flags.mag_aligned_in_flight);
	EXPECT_EQ(0, (int) control_status.flags.ev_vel);
	EXPECT_EQ(0, (int) control_status.flags.synthetic_mag_z);
}

TEST_F(EkfInitializationTest, accleBiasEstimation)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input for 2s
	// WHEN: Added more sensor measurements with accel bias and gps measurements
	Vector3f accel_bias = {0.0f,0.0f,0.1f};

	_sensor_simulator.startGps();
	_sensor_simulator.setImuBias(accel_bias, Vector3f{0.0f,0.0f,0.0f});
	_sensor_simulator.run_seconds(10);

	float converged_pos[3];
	float converged_vel[3];
	float converged_accel_bias[3];
	float converged_gyro_bias[3];
	_ekf->get_position(converged_pos);
	_ekf->get_velocity(converged_vel);
	_ekf->get_accel_bias(converged_accel_bias);
	_ekf->get_gyro_bias(converged_gyro_bias);

	// THEN: EKF should estimate bias correctelly
	for(int i=0; i<3; ++i)
	{
		EXPECT_NEAR(0.0f,converged_pos[i],0.001f) << "i: " << i;
		EXPECT_NEAR(0.0f,converged_vel[i],0.001f) << "i: " << i;
		EXPECT_NEAR(accel_bias(i),converged_accel_bias[i],0.001f) << "i: " << i;
		EXPECT_NEAR(0.0f,converged_gyro_bias[i],0.001f) << "i: " << i;
	}
}

// TODO: Add sampling tests
