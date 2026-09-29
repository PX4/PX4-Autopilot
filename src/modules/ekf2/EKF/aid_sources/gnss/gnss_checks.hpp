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

	/*
	 * Return true if the GNSS solution quality is adequate.
	*/
	bool run(const gnssSample &gnss, uint64_t time_us);
	bool passed() const { return _passed; }
	bool initialChecksPassed() const { return _initial_checks_passed; }

	// How long the checks must pass after a failure before passed() is true
	uint64_t getRequiredPassDurationUs() const
	{
		return _initial_checks_passed ? math::max((uint64_t)1e6, (uint64_t)_params.min_health_time_us / 10)
		       : (uint64_t)_params.min_health_time_us;
	}

	static constexpr uint8_t kNumChecks = estimator_status_s::GPS_CHECK_FAIL_JAMMED + 1;

	// Indexed by estimator_status_s::GPS_CHECK_FAIL_*
	uint16_t getFailFlags() const { return _fail_flags; }

	// The checks enabled in EKF2_GPS_CHECK, indexed by estimator_status_s::GPS_CHECK_FAIL_*
	uint16_t getEnabledChecks() const;

	float horizontal_position_drift_rate_m_s() const { return _horizontal_position_drift_rate_m_s; }
	float vertical_position_drift_rate_m_s() const { return _vertical_position_drift_rate_m_s; }
	float filtered_horizontal_velocity_m_s() const { return _filtered_horizontal_velocity_m_s; }

	static constexpr uint8_t kNoParamBit = 31;

	// EKF2_GPS_CHECK bit of an estimator_status_s::GPS_CHECK_FAIL_* check. The two orders differ, and saved parameters,
	// logs and events each depend on one of them, so neither can be reordered.
	static constexpr uint8_t paramBit(uint8_t check)
	{
		switch (check) {
		case estimator_status_s::GPS_CHECK_FAIL_MIN_SAT_COUNT:    return 0;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_PDOP:         return 1;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_ERR:     return 2;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_ERR:     return 3;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR:      return 4;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_DRIFT:   return 5;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_DRIFT:   return 6;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR: return 7;

		case estimator_status_s::GPS_CHECK_FAIL_MAX_VERT_SPD_ERR: return 8;

		case estimator_status_s::GPS_CHECK_FAIL_SPOOFED:          return 9;

		case estimator_status_s::GPS_CHECK_FAIL_GPS_FIX:          return 10;

		case estimator_status_s::GPS_CHECK_FAIL_JAMMED:           return 11;

		default:                                                  return kNoParamBit;
		}
	}

	bool isCheckEnabled(uint8_t check) const
	{
		return (static_cast<uint32_t>(_params.check_mask) & (1u << paramBit(check))) != 0;
	}

private:
	// A receiver that has not passed for this long qualifies from scratch again, as after a failure. 7 s is the
	// outage after which the EKF stops using GNSS, and after which it used to reset these checks.
	static constexpr uint64_t kPassTimeoutUs = 7'000'000;

	void reset()
	{
		_passed = false;
		_time_last_pass_us = 0;
		_time_last_fail_us = 0;
		resetDriftFilters();
	}

	void setFail(uint8_t check, bool failed);
	bool enabledChecksPass(uint16_t checks) const { return (_fail_flags & checks & getEnabledChecks()) == 0; }

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
