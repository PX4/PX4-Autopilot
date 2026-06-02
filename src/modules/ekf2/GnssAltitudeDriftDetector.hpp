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

#include <stdint.h>

class GnssAltitudeDriftDetector
{
public:
	static constexpr int kWindowSize = 20; // roughly 20 seconds (at 1 Hz sample rate)
	static constexpr int kStabilityWindow = 5; // samples to check for re-enable
	static constexpr float kDriftThreshold = 1.f; // [m]

	// _vel_integral below is the integral of the GPS reported vertical velocity (NED down),
	// not the EKF velocity state. It is the drift-independent altitude reference used for
	// cross-checking against the fused AMSL altitude.
	void update(uint64_t gps_time_us, float gps_vel_d_m_s, float ekf_amsl, float baro_alt);
	void reset();

	bool altitudeGoodForLock() const { return _altitude_good_for_lock; }

	// Cumulative offset since the last reset. Consumers apply the delta against the
	// previously consumed total so missed messages don't desynchronise.
	float accumulatedAltitudeOffset() const { return _accumulated_offset; }
	bool correctionUpdated() const { return _correction_updated; }
	void markCorrectionConsumed() { _correction_updated = false; }

private:
	void analyze();
	void accumulateCorrection(float offset);

	uint64_t _last_gps_ts{0};

	float _d1[kWindowSize] {}; // ekf_amsl - baro_alt
	float _d2[kWindowSize] {}; // ekf_amsl - vel_integral

	float _vel_integral{0.f};
	float _accumulated_offset{0.f};
	int _wcount{0};
	int _widx{0};
	uint64_t _last_sample_ts{0};
	bool _hit_pending{false};
	bool _altitude_good_for_lock{true};
	bool _correction_updated{false};
};
