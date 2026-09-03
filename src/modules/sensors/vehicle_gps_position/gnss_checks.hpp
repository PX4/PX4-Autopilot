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

struct gnssChecksSample {
	uint64_t    time_us{};    ///< timestamp of the measurement (uSec)
	double      lat{};        ///< latitude (degrees)
	double      lon{};        ///< longitude (degrees)
	float       alt{};        ///< GNSS altitude above MSL (m)
	matrix::Vector3f    vel{};        ///< NED earth frame GNSS velocity measurement (m/sec)
	float       hacc{};       ///< 1-std horizontal position error (m)
	float       vacc{};       ///< 1-std vertical position error (m)
	float       sacc{};       ///< 1-std speed error (m/sec)
	uint8_t     fix_type{};   ///< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	uint8_t     nsats{};      ///< number of satellites used
	float       pdop{};       ///< position dilution of precision
	bool        spoofed{};    ///< true if GNSS data is spoofed
	bool        jammed{};     ///< true if GNSS data is jammed
};

union gnssChecks {
	struct {
		uint32_t fix    : 1; ///< 0 - true if the fix type is insufficient (no 3D solution)
		uint32_t nsats  : 1; ///< 1 - true if number of satellites used is insufficient
		uint32_t pdop   : 1; ///< 2 - true if horizontal position dilution of precision is insufficient
		uint32_t hacc   : 1; ///< 3 - true if reported horizontal accuracy is insufficient
		uint32_t vacc   : 1; ///< 4 - true if reported vertical accuracy is insufficient
		uint32_t sacc   : 1; ///< 5 - true if reported speed accuracy is insufficient
		uint32_t hdrift : 1; ///< 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
		uint32_t vdrift : 1; ///< 7 - true if vertical drift is excessive (can only be used when stationary on ground)
		uint32_t hspeed : 1; ///< 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
		uint32_t vspeed : 1; ///< 9 - true if vertical speed error is excessive
		uint32_t spoofed: 1; ///< 10 - true if the GNSS data is spoofed
		uint32_t jammed : 1; ///< 11 - true if the GNSS data is jammed
	} flags;
	uint32_t value;
};

class GnssChecks final
{
public:
	GnssChecks() = default;
	~GnssChecks() = default;

	void setParams(int32_t check_mask, int32_t req_nsats, float req_pdop, float req_eph, float req_epv,
		   float req_sacc, float req_hdrift, float req_vdrift, int32_t req_fix, float vel_lim,
		   uint32_t min_health_time_us);
	/**
	 * Fail-status flags (gnssChecks layout) of the checks enabled by GPS_CHECK.
	 * The param bit order (GnssChecksMask) and the status bit order are not parallel, so they are
	 * mapped one by one: a positional shift silently misassigns every check that is appended to
	 * one of the two enums but not the other.
	 */
	uint16_t getEnabledChecksFailStatusMask() const
	{
		gnssChecks mask{};
		mask.flags.fix     = isCheckEnabled(GnssChecksMask::kFix);
		mask.flags.nsats   = isCheckEnabled(GnssChecksMask::kNsats);
		mask.flags.pdop    = isCheckEnabled(GnssChecksMask::kPdop);
		mask.flags.hacc    = isCheckEnabled(GnssChecksMask::kHacc);
		mask.flags.vacc    = isCheckEnabled(GnssChecksMask::kVacc);
		mask.flags.sacc    = isCheckEnabled(GnssChecksMask::kSacc);
		mask.flags.hdrift  = isCheckEnabled(GnssChecksMask::kHdrift);
		mask.flags.vdrift  = isCheckEnabled(GnssChecksMask::kVdrift);
		mask.flags.hspeed  = isCheckEnabled(GnssChecksMask::kHspd);
		mask.flags.vspeed  = isCheckEnabled(GnssChecksMask::kVspd);
		mask.flags.spoofed = isCheckEnabled(GnssChecksMask::kSpoofed);
		mask.flags.jammed  = isCheckEnabled(GnssChecksMask::kJammed);
		return mask.value;
	}

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
	bool run(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest);
	bool passed() const { return _passed; }
	bool initialChecksPassed() const { return _initial_checks_passed; }
	uint64_t getLastPassUs() const { return _time_last_pass_us; }
	uint64_t getLastFailUs() const { return _time_last_fail_us; }

	const gnssChecks &getFailStatus() const { return _check_fail_status; }

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
		kFix     = (1 << 10),
		kJammed  = (1 << 11)
	};

	bool isCheckEnabled(GnssChecksMask check) const { return (_params.check_mask & static_cast<int32_t>(check)); }

	bool runSimplifiedChecks(const gnssChecksSample &gnss);
	bool runInitialFixChecks(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest);
	void runOnGroundGnssChecks(const gnssChecksSample &gnss, bool in_air, bool vehicle_at_rest);

	void resetDriftFilters();

	bool isTimedOut(uint64_t timestamp_to_check_us, uint64_t now_us, uint64_t timeout_period) const
	{
		return (timestamp_to_check_us == 0) || (timestamp_to_check_us + timeout_period < now_us);
	}

	gnssChecks _check_fail_status{};

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
	bool _passed{false};

	struct Params {
		int32_t check_mask;
		int32_t req_nsats;
		float req_pdop;
		float req_eph;
		float req_epv;
		float req_sacc;
		float req_hdrift;
		float req_vdrift;
		int32_t req_fix;
		float vel_lim;
		uint32_t min_health_time_us;
	};

	Params _params;
};

#endif // !GNSS_CHECKS_H
