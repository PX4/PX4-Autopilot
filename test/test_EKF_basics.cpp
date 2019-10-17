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
#include <cmath>
#include "EKF/ekf.h"

class EkfInitializationTest : public ::testing::Test {
 public:

	Ekf _ekf{};

	// Basics sensors
	const uint32_t _imu_dt_us{4000};	// 250 Hz	Period between IMU updates
	const uint32_t _baro_dt_us{12500};	// 80 Hz	Period between barometer updates
	const uint32_t _mag_dt_us{12500};	// 80 Hz	Period between magnetometer updates
	const uint32_t _gps_dt_us{200000};	// 5 Hz		Period between GPS updates

	// Flags that control if a sensor is fused
	bool _fuse_imu{true};
	bool _fuse_baro{true};
	bool _fuse_mag{true};
	bool _fuse_gps{false};	// GPS measurements are expected to not come in from beginning

	// GPS message
	gps_message _gps_message{};

	uint32_t _update_dt_us{};			// greatest common divider of all basic sensor periods
	const uint32_t _init_duration_us{2000000};	// 2s	Duration of

	// counter of how many sensor measurement are put into Ekf
	uint32_t _counter_imu{0};
	uint32_t _counter_baro{0};
	uint32_t _counter_mag{0};

	uint32_t _t_us{0};

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{

		_ekf.init(0);

		// setup gps message to reasonable default values
		_gps_message.time_usec = 0;
		_gps_message.lat = 473566094;
		_gps_message.lon = 85190237;
		_gps_message.alt = 422056;
		_gps_message.yaw = 0.0f;
		_gps_message.yaw_offset = 0.0f;
		_gps_message.fix_type = 3;
		_gps_message.eph = 0.5f;
		_gps_message.epv = 0.8f;
		_gps_message.sacc = 0.2f;
		_gps_message.vel_m_s = 0.0;
		_gps_message.vel_ned[0] = 0.0f;
		_gps_message.vel_ned[1] = 0.0f;
		_gps_message.vel_ned[2] = 0.0f;
		_gps_message.vel_ned_valid = 1;
		_gps_message.nsats = 16;
		_gps_message.gdop = 0.0f;

		update_with_const_sensors(_init_duration_us);

		// output how many sensor measurement were put into the EKF
		// std::cout << "Initialized EKF with:" << std::endl;
		// std::cout << "update_dt_us: " << _update_dt_us << std::endl;
		// std::cout << "counter_imu:  " << _counter_imu << std::endl
		// 	  << "counter_baro: " << _counter_baro << std::endl
		// 	  << "counter_mag:  " << _counter_mag << std::endl;
	}

	void update_with_const_sensors(uint32_t duration_us,
			Vector3f ang_vel = Vector3f{0.0f,0.0f,0.0f},
			Vector3f accel = Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G},
			Vector3f mag_data = Vector3f{0.2f, 0.0f, 0.4f},
			float baro_data = 122.2f)
	{
		// store start time
		uint32_t start_time_us = _t_us;

		// compute update time step such that we can update the basic sensor at different rates
		_update_dt_us = std::__gcd(_imu_dt_us,std::__gcd(_mag_dt_us,std::__gcd(_baro_dt_us,_gps_dt_us)));

		// update EKF with synthetic sensor measurements
		for( ; _t_us < start_time_us+duration_us; _t_us += _update_dt_us)
		{
			// Check which sensors update we should do
			if(_fuse_imu && !(_t_us %_imu_dt_us))
			{
				// push imu data into estimator
				imuSample imu_sample_new;
				imu_sample_new.time_us = _t_us;
				imu_sample_new.delta_ang_dt = _imu_dt_us * 1.e-6f;
				imu_sample_new.delta_ang = ang_vel * imu_sample_new.delta_ang_dt;
				imu_sample_new.delta_vel_dt = _imu_dt_us * 1.e-6f;
				imu_sample_new.delta_vel = accel * imu_sample_new.delta_vel_dt;

				_ekf.setIMUData(imu_sample_new);
				_counter_imu++;
			}
			if(_fuse_baro && !(_t_us % _baro_dt_us))
			{
				_ekf.setBaroData(_t_us,baro_data);
				_counter_baro++;
			}
			if(_fuse_mag && !(_t_us % _mag_dt_us))
			{
				float mag[3];
				mag_data.copyTo(mag);
				_ekf.setMagData(_t_us,mag);
				_counter_mag++;
			}
			if(_fuse_gps && !(_t_us % _gps_dt_us))
			{
				_gps_message.time_usec = _t_us;
				_ekf.setGpsData(_t_us,_gps_message);
				_counter_mag++;
			}

			_ekf.update();
		}
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
	EXPECT_EQ(true,_ekf.attitude_valid());
}

TEST_F(EkfInitializationTest, initialControlMode)
{
	// GIVEN: reasonable static sensor data for some duration
	// THEN: EKF control status should be reasonable
	filter_control_status_u control_status;
	_ekf.get_control_mode(&control_status.value);

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
	EXPECT_EQ(0, (int) control_status.flags.update_mag_states_only);
	EXPECT_EQ(0, (int) control_status.flags.fixed_wing);
	EXPECT_EQ(0, (int) control_status.flags.mag_fault);
	EXPECT_EQ(0, (int) control_status.flags.gnd_effect);
	EXPECT_EQ(0, (int) control_status.flags.rng_stuck);
	EXPECT_EQ(0, (int) control_status.flags.gps_yaw);
	EXPECT_EQ(0, (int) control_status.flags.mag_align_complete);
	EXPECT_EQ(0, (int) control_status.flags.ev_vel);
	EXPECT_EQ(0, (int) control_status.flags.synthetic_mag_z);
}

TEST_F(EkfInitializationTest, convergesToZero)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input for 2s
	// WHEN: Added more defautl sensor measurements
	update_with_const_sensors(4000000); // for further 4s

	float converged_pos[3];
	float converged_vel[3];
	float converged_accel_bias[3];
	float converged_gyro_bias[3];
	_ekf.get_position(converged_pos);
	_ekf.get_velocity(converged_vel);
	_ekf.get_accel_bias(converged_accel_bias);
	_ekf.get_gyro_bias(converged_gyro_bias);

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
	// GIVEN: initialized EKF with default IMU, baro and mag input for 2s
	// WHEN: setting GPS measurements for 11s, minimum GPS health time is set to 10 sec

	_fuse_gps = true;
	update_with_const_sensors(11000000,Vector3f{0.0f,0.0f,0.0f},Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G}); // for further 3s

	// THEN: EKF should fuse GPS, but no other position sensor
	filter_control_status_u control_status;
	_ekf.get_control_mode(&control_status.value);
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
	EXPECT_EQ(0, (int) control_status.flags.update_mag_states_only);
	EXPECT_EQ(0, (int) control_status.flags.fixed_wing);
	EXPECT_EQ(0, (int) control_status.flags.mag_fault);
	EXPECT_EQ(0, (int) control_status.flags.gnd_effect);
	EXPECT_EQ(0, (int) control_status.flags.rng_stuck);
	EXPECT_EQ(0, (int) control_status.flags.gps_yaw);
	EXPECT_EQ(0, (int) control_status.flags.mag_align_complete);
	EXPECT_EQ(0, (int) control_status.flags.ev_vel);
	EXPECT_EQ(0, (int) control_status.flags.synthetic_mag_z);
}

TEST_F(EkfInitializationTest, accleBiasEstimation)
{
	// GIVEN: initialized EKF with default IMU, baro and mag input for 2s
	// WHEN: Added more sensor measurements with accel bias and gps measurements
	Vector3f accel_bias = {0.0f,0.0f,0.1f};

	_fuse_gps = true;
	update_with_const_sensors(10000000,Vector3f{0.0f,0.0f,0.0f},Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G}+accel_bias); // for further 10s

	float converged_pos[3];
	float converged_vel[3];
	float converged_accel_bias[3];
	float converged_gyro_bias[3];
	_ekf.get_position(converged_pos);
	_ekf.get_velocity(converged_vel);
	_ekf.get_accel_bias(converged_accel_bias);
	_ekf.get_gyro_bias(converged_gyro_bias);

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
