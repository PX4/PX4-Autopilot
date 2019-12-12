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

/**
 * This class is providing methods to feed the ECL EKF with measurement.
 * It takes a pointer to the Ekf object and will manipulate the object
 * by call set*Data functions.
 * It simulates the time to allow for sensor data being set at certain rate
 * and also calls the update method of the EKF
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#pragma once

#include "EKF/ekf.h"
#include <math.h>


namespace simulator
{

class SensorSimulator
{
public:
	SensorSimulator(Ekf* ekf);
	~SensorSimulator();

	void update_with_const_sensors(uint32_t duration_us,
		Vector3f ang_vel = Vector3f{0.0f,0.0f,0.0f},
		Vector3f accel = Vector3f{0.0f,0.0f,-CONSTANTS_ONE_G},
		Vector3f mag_data = Vector3f{0.2f, 0.0f, 0.4f},
		float baro_data = 122.2f);

	void setGpsFusionTrue(){ _fuse_gps = true; }
	void setGpsFusionFalse(){ _fuse_gps = false; }

private:
	Ekf* _ekf;

	// current time
	uint32_t _t_us{0};

	// Basics sensors
	const uint32_t _imu_dt_us{4000};	// 250 Hz	Period between IMU updates
	const uint32_t _baro_dt_us{12500};	// 80 Hz	Period between barometer updates
	const uint32_t _mag_dt_us{12500};	// 80 Hz	Period between magnetometer updates
	const uint32_t _gps_dt_us{200000};	// 5 Hz		Period between GPS updates
	// const uint32_t _flow_dt_us{20000};	// 50 Hz	Period between Flow updates
	// const uint32_t _ev_dt_us{40000};	// 25 Hz	Period between external vision updates

	uint32_t _update_dt_us{};			// greatest common divider of all basic sensor periods


	// Flags that control if a sensor is fused
	bool _fuse_imu{true};
	bool _fuse_baro{true};
	bool _fuse_mag{true};
	// Not expected to be fused from beginning
	bool _fuse_gps{false};
	// bool _fuse_flow{false};
	// bool _fuse_ev{false};

	gps_message _gps_message{};

	// used for debugging until now, replace with tests
	// counter of how many sensor measurement are put into Ekf
	uint32_t _counter_imu{0};
	uint32_t _counter_baro{0};
	uint32_t _counter_mag{0};


	void setGpsMessageToDefaul();



};

// Compute greatest common divider
inline uint32_t gcd(uint32_t a, uint32_t b)
{
	return b == 0 ? a : gcd(b, a % b);
}

} // end of namespace
