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

#include "UtcToHrtMapper.hpp"

hrt_abstime UtcToHrtMapper::map(uint64_t utc_us, hrt_abstime hrt_recv_us)
{
	if (utc_us == 0) {
		return 0;
	}

	const int64_t cand_offset = static_cast<int64_t>(utc_us) - static_cast<int64_t>(hrt_recv_us);

	// Hard reset on a UTC discontinuity: a backward jump in cand larger than
	// any plausible latency spike means the previous offset estimate is stale.
	if (_valid && cand_offset < _offset_us - kUtcDiscontinuityResetUs) {
		_valid = false;
		_last_emitted_us = 0;
	}

	if (!_valid) {
		_offset_us = cand_offset;
		_valid = true;

	} else {
		const hrt_abstime dt_us = (hrt_recv_us > _last_update_us) ? (hrt_recv_us - _last_update_us) : 0;
		const int64_t decay_us = static_cast<int64_t>(dt_us) * kMaxClockDriftPpm / 1'000'000;
		_offset_us -= decay_us;

		if (cand_offset > _offset_us) {
			_offset_us = cand_offset;
		}
	}

	_last_update_us = hrt_recv_us;

	int64_t pvt_hrt_us = static_cast<int64_t>(utc_us) - _offset_us;

	// Strict-less-than hrt_recv_us: equality is misread upstream as
	// "driver didn't set timestamp_sample" and replaced with SENS_GPS_DELAY.
	if (pvt_hrt_us >= static_cast<int64_t>(hrt_recv_us)) {
		pvt_hrt_us = static_cast<int64_t>(hrt_recv_us) - 1;
	}

	// Strict-greater-than _last_emitted_us: snap-ups during the convergence
	// transient can produce a backward step that the EKF GPS buffer rejects.
	if (_last_emitted_us > 0 && pvt_hrt_us <= static_cast<int64_t>(_last_emitted_us)) {
		pvt_hrt_us = static_cast<int64_t>(_last_emitted_us) + 1;
	}

	// Both clamps applied; bail out if the window (_last_emitted_us, hrt_recv_us)
	// was too narrow to satisfy them.
	if (pvt_hrt_us <= 0 || pvt_hrt_us >= static_cast<int64_t>(hrt_recv_us)) {
		return 0;
	}

	_last_emitted_us = static_cast<hrt_abstime>(pvt_hrt_us);
	return _last_emitted_us;
}
