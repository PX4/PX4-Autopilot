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

#ifndef EKF_GNSS_CHECKS_H
#define EKF_GNSS_CHECKS_H

#include <lib/geo/geo.h>

#include "../../common.h"

namespace estimator
{
class GnssChecks final
{
public:
	GnssChecks(int32_t &check_mask, int32_t &ekf2_req_nsats, float &ekf2_req_pdop, float &ekf2_req_eph, float &ekf2_req_epv,
		   float &ekf2_req_sacc, float &ekf2_req_hdrift, float &ekf2_req_vdrift, int32_t &ekf2_req_fix, float &ekf2_vel_lim,
		   uint32_t &min_health_time_us, filter_control_status_u &control_status):
		_params{check_mask, ekf2_req_nsats, ekf2_req_pdop, ekf2_req_eph, ekf2_req_epv, ekf2_req_sacc, ekf2_req_hdrift, ekf2_req_vdrift, ekf2_req_fix, ekf2_vel_lim, min_health_time_us},
		_control_status(control_status)
	{};

	union gps_check_fail_status_u {
		struct {
			uint16_t fix    : 1; ///< 0 - true if the fix type is insufficient (no 3D solution)
			uint16_t nsats  : 1; ///< 1 - true if number of satellites used is insufficient
			uint16_t pdop   : 1; ///< 2 - true if position dilution of precision is insufficient
			uint16_t hacc   : 1; ///< 3 - true if reported horizontal accuracy is insufficient
			uint16_t vacc   : 1; ///< 4 - true if reported vertical accuracy is insufficient
			uint16_t sacc   : 1; ///< 5 - true if reported speed accuracy is insufficient
			uint16_t hdrift : 1; ///< 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
			uint16_t vdrift : 1; ///< 7 - true if vertical drift is excessive (can only be used when stationary on ground)
			uint16_t hspeed : 1; ///< 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
			uint16_t vspeed : 1; ///< 9 - true if vertical speed error is excessive
			uint16_t spoofed: 1; ///< 10 - true if the GNSS data is spoofed
		} flags;
		uint16_t value;
	};

	void resetHard()
	{
		_initial_checks_passed = false;
		reset();
	}

	void reset()
	{
		_passed = false;
		_time_last_pass_us = 0;
		_time_last_fail_us = 0;
		resetDriftFilters();
	}

	/*
	 * Return true if the GNSS solution quality is adequate.
	*/
	bool run(const gnssSample &gnss, uint64_t time_us);
	bool passed() const { return _passed; }
	bool initialChecksPassed() const { return _initial_checks_passed; }
	uint64_t getLastPassUs() const { return _time_last_pass_us; }
	uint64_t getLastFailUs() const { return _time_last_fail_us; }

	const gps_check_fail_status_u &getFailStatus() const { return _check_fail_status; }

	float horizontal_position_drift_rate_m_s() const { return _horizontal_position_drift_rate_m_s; }
	float vertical_position_drift_rate_m_s() const { return _vertical_position_drift_rate_m_s; }
	float filtered_horizontal_velocity_m_s() const { return _filtered_horizontal_velocity_m_s; }

private:
	enum class GnssChecksMask : int32_t {
		kNsats   = (1 << 0),
		kPdop    = (1 << 1),
		kHacc    = (1 << 2),
		kVacc    = (1 << 3),
		kSacc    = (1 << 4),
		kHdrift  = (1 << 5),
		kVdrift  = (1 << 6),
		kHspd    = (1 << 7),
		kVspd    = (1 << 8),
		kSpoofed = (1 << 9),
		kFix     = (1 << 10)
	};

	bool isCheckEnabled(GnssChecksMask check) { return (_params.check_mask & static_cast<int32_t>(check)); }

	bool runSimplifiedChecks(const gnssSample &gnss);
	bool runInitialFixChecks(const gnssSample &gnss);
	void runOnGroundGnssChecks(const gnssSample &gnss);

	void resetDriftFilters();

	bool isTimedOut(uint64_t timestamp_to_check_us, uint64_t now_us, uint64_t timeout_period) const
	{
		return (timestamp_to_check_us == 0) || (timestamp_to_check_us + timeout_period < now_us);
	}

	gps_check_fail_status_u _check_fail_status{};

	float _horizontal_position_drift_rate_m_s{NAN};
	float _vertical_position_drift_rate_m_s{NAN};
	float _filtered_horizontal_velocity_m_s{NAN};

	MapProjection lat_lon_prev{};
	float _alt_prev{0.0f};

	Vector3f _lat_lon_alt_deriv_filt{};
	Vector2f _vel_ne_filt{};

	float _vel_d_filt{0.0f};		///< GNSS filtered Down velocity (m/sec)
	uint64_t _time_last_fail_us{0};
	uint64_t _time_last_pass_us{0};
	bool _initial_checks_passed{false};
	bool _passed{false};

	struct Params {
		const int32_t &check_mask;
		const int32_t &ekf2_req_nsats;
		const float &ekf2_req_pdop;
		const float &ekf2_req_eph;
		const float &ekf2_req_epv;
		const float &ekf2_req_sacc;
		const float &ekf2_req_hdrift;
		const float &ekf2_req_vdrift;
		const int32_t &ekf2_req_fix;
		const float &ekf2_vel_lim;
		const uint32_t &min_health_time_us;
	};

	const Params _params;
	const filter_control_status_u &_control_status;
};
}; // namespace estimator

#endif // !EKF_GNSS_CHECKS_H
