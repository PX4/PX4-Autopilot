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

#ifndef GNSS_CHECKS_H
#define GNSS_CHECKS_H

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_gnss.h>

struct gnssChecksSample {
	uint64_t time_us{};     ///< measurement time (us)
	double lat{};           ///< latitude (deg)
	double lon{};           ///< longitude (deg)
	float alt{};            ///< altitude above MSL (m)
	matrix::Vector3f vel{}; ///< NED velocity (m/s)
	float hacc{};           ///< 1-std horizontal position error (m)
	float vacc{};           ///< 1-std vertical position error (m)
	float sacc{};           ///< 1-std speed error (m/s)
	uint8_t fix_type{};     ///< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: RTK float, 6: RTK fixed
	uint8_t nsats{};        ///< number of satellites used
	float pdop{};           ///< position dilution of precision
	bool spoofed{};         ///< the receiver reports spoofing
	bool jammed{};          ///< the receiver reports jamming
};

class GnssChecks final
{
public:
	struct Params {
		int32_t check_mask{2047};
		int32_t req_nsats{6};
		float req_pdop{2.5f};
		float req_eph{3.f};
		float req_epv{5.f};
		float req_sacc{0.5f};
		float req_hdrift{0.1f};
		float req_vdrift{0.2f};
		int32_t req_fix{3};
		uint64_t min_health_time_us{10'000'000};
	};

	void setParams(const Params &params) { _params = params; }

	/*
	 * Return true if the GNSS solution quality is adequate. The strict checks apply until the first pass and again
	 * whenever the vehicle is disarmed on the ground; the drift checks run only at rest on the ground.
	*/
	bool run(const gnssChecksSample &gnss, bool armed, bool in_air, bool vehicle_at_rest);
	bool passed() const { return _passed; }

	// The strict checks decided the last run: never passed yet, disarmed on the ground, or passing them
	bool strict() const { return _strict; }

	// The last sample passed the strict checks enabled in the check mask, and no check that decided passed() failed for
	// the required pass duration. Once the relaxed checks apply, a sample that fails only the strict ones doesn't restart
	// that duration, so that a single one doesn't drop the receiver's rank for GNSS_REQ_TIME: the selection's own hold
	// rides through it. Drift is evaluated only at rest on the ground.
	bool meetsRequirements() const { return _meets_requirements; }

	// Failed checks, as vehicle_gnss_s::CHECK_* bits
	uint16_t getFailFlags() const { return _fail_flags; }

	// The checks enabled in GNSS_CHECK, as vehicle_gnss_s::CHECK_* bits
	uint16_t getEnabledChecks() const { return static_cast<uint16_t>(_params.check_mask) & kAllChecks; }

	float horizontal_position_drift_rate_m_s() const { return _horizontal_position_drift_rate_m_s; }
	float vertical_position_drift_rate_m_s() const { return _vertical_position_drift_rate_m_s; }
	float filtered_horizontal_velocity_m_s() const { return _filtered_horizontal_velocity_m_s; }

private:
	static constexpr uint16_t kAllChecks = vehicle_gnss_s::CHECK_NSATS | vehicle_gnss_s::CHECK_PDOP | vehicle_gnss_s::CHECK_EPH
					       | vehicle_gnss_s::CHECK_EPV | vehicle_gnss_s::CHECK_SACC | vehicle_gnss_s::CHECK_HDRIFT
					       | vehicle_gnss_s::CHECK_VDRIFT | vehicle_gnss_s::CHECK_HSPEED | vehicle_gnss_s::CHECK_VSPEED
					       | vehicle_gnss_s::CHECK_SPOOFED | vehicle_gnss_s::CHECK_FIX | vehicle_gnss_s::CHECK_JAMMED;

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

	// How long the checks must pass after a failure before passed() is true
	uint64_t getRequiredPassDurationUs(const bool simplified = false) const
	{
		return simplified ? math::max((uint64_t)1e6, (uint64_t)_params.min_health_time_us / 10)
		       : (uint64_t)_params.min_health_time_us;
	}

	void setFail(uint16_t check, bool failed);
	bool enabledChecksPass(uint16_t checks) const { return (_fail_flags & checks & getEnabledChecks()) == 0; }

	bool runSimplifiedChecks(const gnssChecksSample &gnss);
	bool runInitialFixChecks(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest);
	void runOnGroundGnssChecks(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest);

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

	matrix::Vector3f _lat_lon_alt_deriv_filt{};
	matrix::Vector2f _vel_ne_filt{};

	float _vel_d_filt{0.0f};		///< GNSS filtered Down velocity (m/sec)
	uint64_t _time_last_fail_us{0};
	uint64_t _time_last_pass_us{0};
	bool _initial_checks_passed{false};
	bool _strict{true};
	bool _passed{false};
	bool _meets_requirements{false};

	Params _params{};
};

#endif // !GNSS_CHECKS_H
