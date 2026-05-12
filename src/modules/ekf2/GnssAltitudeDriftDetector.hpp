/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "mathlib/math/filter/AlphaFilter.hpp"

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/gnss_altitude_drift.h>
#include <uORB/topics/sensor_gps.h>

class GnssAltitudeDriftDetector
{
public:
	static constexpr int kWindowSize = 20; // roughly 20 seconds (at 1 Hz sample rate)
	static constexpr int kStabilityWindow = 5; // samples to check for re-enable
	static constexpr float kDriftThreshold = 1.f; // [m]

	void updateBaroLpf(float baro_alt, uint64_t timestamp);
	void update(const sensor_gps_s &gps, float ekf_amsl);
	void reset();

	bool _altitude_good_for_lock{true};

private:
	void analyze();
	void publishCorrection(float offset);

	uORB::PublicationMulti<gnss_altitude_drift_s> _gnss_altitude_drift_pub{ORB_ID(gnss_altitude_drift)};

	AlphaFilter<float> _baro_lpf;
	uint64_t _last_baro_ts{0};
	uint64_t _last_gps_ts{0};

	float _d1[kWindowSize] {}; // ekf_amsl - baro_alt
	float _d2[kWindowSize] {}; // ekf_amsl - vel_integral

	float _vel_integral{0.f};
	int _wcount{0};
	int _widx{0};
	uint64_t _last_sample_ts{0};
	bool _hit_pending{false};
};
