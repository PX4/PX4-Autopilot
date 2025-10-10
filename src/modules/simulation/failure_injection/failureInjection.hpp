/****************************************************************************
*
*   Copyright (c) 2019-2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/bitmask.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/sensor_gps.h>
#include <lib/geo/geo.h>
#include <matrix/math.hpp>

static constexpr uint8_t kAccelCountMax = 4;
static constexpr uint8_t kGyroCountMax  = 4;
static constexpr uint8_t kMagCountMax   = 4;

static constexpr float kRelativeGpsDrift = 0.1f;

class FailureInjection
{

public:
	FailureInjection() = default;

	void check_failure_injections();
	bool handle_gps_failure(sensor_gps_s &gps);
	bool handle_gps_alt_failure(sensor_gps_s &gps);

	bool is_accel_blocked(const int instance = 0) { return _accel_blocked[instance]; }
	bool is_accel_stuck(const int instance = 0) { return _accel_stuck[instance]; }

	bool is_gyro_blocked(const int instance = 0) { return _gyro_blocked[instance]; }
	bool is_gyro_stuck(const int instance = 0) { return _gyro_stuck[instance]; }

	bool is_mag_blocked(const int instance = 0) { return _mag_blocked[instance]; }
	bool is_mag_stuck(const int instance = 0) { return _mag_stuck[instance]; }

	bool is_baro_blocked() { return _baro_blocked; }
	bool is_baro_stuck() { return _baro_stuck; }

	bool is_airspeed_disconnected() { return _airspeed_disconnected; }
	hrt_abstime get_airspeed_blocked_timestamp() { return _airspeed_blocked_timestamp; }

	bool is_vio_blocked() { return _vio_blocked; }

	bool is_agp_blocked() { return _agp_blocked; }
	bool is_agp_stuck() { return _agp_stuck; }
	bool is_agp_drift() { return _agp_drift; }

	bool is_distance_sensor_blocked() { return _distance_sensor_blocked; }
	bool is_distance_sensor_stuck() { return _distance_sensor_stuck; }
	bool is_distance_sensor_wrong() { return _distance_sensor_wrong; }

private:

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	bool _gps_drift{false};
	bool _gps_alt_drift{false};
	MapProjection _mp;
	bool _has_drift_ref{false};
	matrix::Vector2f _gps_drift_pos;
	double _gps_alt_offset{0};
	hrt_abstime _last_gps_timestamp{0};
	hrt_abstime _alt_drift_t0{0};

	bool _gps_blocked{false};
	bool _gps_alt_blocked{false};
	bool _gps_stuck{false};
	bool _gps_alt_stuck{false};
	bool _gps_wrong{false};
	bool _gps_alt_wrong{false};
	sensor_gps_s _gps_prev{};

	bool _accel_blocked[kAccelCountMax] {};
	bool _accel_stuck[kAccelCountMax] {};
	bool _gyro_blocked[kGyroCountMax] {};
	bool _gyro_stuck[kGyroCountMax] {};

	bool _mag_blocked[kMagCountMax] {};
	bool _mag_stuck[kMagCountMax] {};
	bool _baro_blocked{false};
	bool _baro_stuck{false};
	bool _airspeed_disconnected{false};
	hrt_abstime _airspeed_blocked_timestamp{0};
	bool _vio_blocked{false};
	bool _agp_blocked{false};
	bool _agp_stuck{false};
	bool _agp_drift{false};
	bool _distance_sensor_blocked{false};
	bool _distance_sensor_stuck{false};
	bool _distance_sensor_wrong{false};

};
