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

/**
 * @file PpsTimeSync.cpp
 *
 * PPS-based time synchronization implementation
 */

#include "PpsTimeSync.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

void PpsTimeSync::process_pps(const pps_capture_s &pps)
{
	if (pps.timestamp == 0 || pps.rtc_timestamp == 0) {
		return;
	}

	_pps_hrt_timestamp = pps.timestamp;
	_pps_rtc_timestamp = pps.rtc_timestamp;
	_time_offset = (int64_t)pps.rtc_timestamp - (int64_t)pps.timestamp;
	_initialized = true;
	_updated = true;
}

uint64_t PpsTimeSync::correct_gps_timestamp(uint64_t gps_fc_timestamp, uint64_t gps_utc_timestamp)
{
	if (!is_valid()) {
		return gps_fc_timestamp;
	}

	const int64_t corrected_fc_timestamp = (int64_t)gps_utc_timestamp - _time_offset;

	if (_updated) {
		const int64_t correction_amount = corrected_fc_timestamp - (int64_t)gps_fc_timestamp;

		if (math::abs_t(correction_amount) > kPpsMaxCorrectionUs) {
			PX4_DEBUG("PPS: Correction too large: %" PRId64 " us (%.1f ms), rejecting",
				  correction_amount, (double)correction_amount / 1000.0);
			return gps_fc_timestamp;
		}

		// Additional sanity check: corrected timestamp should not be too far in the future (0.1s)
		const uint64_t now = hrt_absolute_time();

		if ((uint64_t)corrected_fc_timestamp > now + 100000) {
			return gps_fc_timestamp;
		}

		_updated = false;
	}

	return (uint64_t)corrected_fc_timestamp;
}

bool PpsTimeSync::is_valid() const
{
	if (!_initialized) {
		return false;
	}

	uint64_t now = hrt_absolute_time();

	if (now < _pps_hrt_timestamp) {
		now = UINT64_MAX;
	}

	return now - _pps_hrt_timestamp < kPpsStaleTimeoutUs;
}
