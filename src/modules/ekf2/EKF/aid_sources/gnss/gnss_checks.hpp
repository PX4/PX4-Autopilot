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
#include <uORB/topics/estimator_status.h>

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

	// Indexed by estimator_status_s::GPS_CHECK_FAIL_*, the same bit positions as EKF2_GPS_CHECK.
	uint16_t getFailFlags() const { return _fail_flags; }

	float horizontal_position_drift_rate_m_s() const { return _horizontal_position_drift_rate_m_s; }
	float vertical_position_drift_rate_m_s() const { return _vertical_position_drift_rate_m_s; }
	float filtered_horizontal_velocity_m_s() const { return _filtered_horizontal_velocity_m_s; }

private:
	void setFail(uint8_t check, bool failed);
	bool enabledChecksPass(uint16_t checks) const { return (_fail_flags & checks & _params.check_mask) == 0; }

	bool runSimplifiedChecks(const gnssSample &gnss);
	bool runInitialFixChecks(const gnssSample &gnss);
	void runOnGroundGnssChecks(const gnssSample &gnss);

	void clearDriftChecks();
	void resetDriftFilters();

	bool isTimedOut(uint64_t timestamp_to_check_us, uint64_t now_us, uint64_t timeout_period) const
	{
		return (timestamp_to_check_us == 0) || (timestamp_to_check_us + timeout_period < now_us);
	}

	uint16_t _fail_flags{0};

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
