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

#include "GnssAltitudeDriftDetector.hpp"

#include <drivers/drv_hrt.h>
#include <math.h>

void GnssAltitudeDriftDetector::updateBaroLpf(float baro_alt, uint64_t timestamp)
{
	const float dt = 1e-6f * (timestamp - _last_baro_ts);

	if (_last_baro_ts != 0 && dt < 1.f) {
		_baro_lpf.update(baro_alt, dt);

	} else {
		_baro_lpf.reset(baro_alt);
	}

	_last_baro_ts = timestamp;
}

void GnssAltitudeDriftDetector::update(const sensor_gps_s &gps, float ekf_amsl)
{
	const bool gps_timeout = (_last_gps_ts != 0) && (gps.timestamp - _last_gps_ts > 500000);

	if (gps_timeout || _last_gps_ts == 0 || _last_baro_ts == 0) {
		reset();
		_last_gps_ts = gps.timestamp;
		return;
	}

	_vel_integral += 1e-6f * (gps.timestamp - _last_gps_ts) * (-gps.vel_d_m_s);
	_last_gps_ts = gps.timestamp;

	// sample at 1Hz normally, or immediately on pending hit
	const bool sample_due = (_last_sample_ts == 0)
				|| _hit_pending
				|| (gps.timestamp >= _last_sample_ts + 1000000);

	if (!sample_due) {
		return;
	}

	_d1[_widx] = ekf_amsl - _baro_lpf.getState();
	_d2[_widx] = ekf_amsl - _vel_integral;
	_widx = (_widx + 1) % kWindowSize;

	if (_wcount < kWindowSize) {
		_wcount++;
	}

	_last_sample_ts = gps.timestamp;

	if (_wcount > 1) {
		analyze();
	}
}

void GnssAltitudeDriftDetector::analyze()
{
	const int newest = (_widx - 1 + kWindowSize) % kWindowSize;
	const int oldest = (_widx - _wcount + kWindowSize) % kWindowSize;

	const float a = fabsf(_d1[newest] - _d1[oldest]); // change in (ekf_amsl - baro_alt) over window
	const float b = fabsf(_d2[newest] - _d2[oldest]); // change in (ekf_amsl - vel_integral) over window
	const float c = fabsf((_d1[newest] - _d2[newest]) - (_d1[oldest] - _d2[oldest])); // change in (vel_integral - baro_alt) over window

	// gps_alt drift has to have relevant magnitude and larger than vel-baro drift
	const bool hit = (a > kDriftThreshold) && (b > kDriftThreshold) && (a > c) && (b > c);

	// hit pending to filter out single outliers
	if (hit && _hit_pending) {
		publishCorrection(_d1[newest] - _d1[oldest]);
		_hit_pending = false;
		_altitude_good_for_lock = false;
		_wcount = 1;

	} else {
		_hit_pending = hit;
	}

	// Re-enable when recent samples show stability
	if (_altitude_good_for_lock || _wcount < kStabilityWindow) {
		return;
	}

	const int recent = (_widx - kStabilityWindow + kWindowSize) % kWindowSize;
	const float a_recent = fabsf(_d1[newest] - _d1[recent]);
	const float b_recent = fabsf(_d2[newest] - _d2[recent]);

	if ((a_recent < 0.2f * kDriftThreshold) && (b_recent < 0.2f * kDriftThreshold)) {
		_altitude_good_for_lock = true;
		const float residual = _d1[newest] - _d1[oldest];

		if (fabsf(residual) > 0.01f) {
			publishCorrection(residual);
		}

		_wcount = 1;
	}
}

void GnssAltitudeDriftDetector::publishCorrection(float offset)
{
	gnss_altitude_drift_s gnss_altitude_drift{};
	gnss_altitude_drift.timestamp = hrt_absolute_time();
	gnss_altitude_drift.altitude_offset = offset;
	_gnss_altitude_drift_pub.publish(gnss_altitude_drift);
}

void GnssAltitudeDriftDetector::reset()
{
	_altitude_good_for_lock = true;
	_vel_integral = 0.f;
	_wcount = 0;
	_widx = 0;
	_last_sample_ts = 0;
	_hit_pending = false;
}
