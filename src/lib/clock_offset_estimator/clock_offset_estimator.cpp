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

#include "clock_offset_estimator.hpp"

hrt_abstime ClockOffsetEstimator::toLocal(uint64_t remote_us, hrt_abstime local_us)
{
	if (remote_us == 0) {
		return 0;
	}

	const int64_t lag = static_cast<int64_t>(local_us) - static_cast<int64_t>(remote_us);

	if (!_initialized || lag > _link_offset_us + kResetThresholdUs) {
		// Bootstrap, or remote-clock discontinuity.
		_link_offset_us = lag;
		_initialized = true;

	} else {
		// Drift the floor up at the worst-case relative clock rate so we don't
		// pin below the true offset when HRT runs faster than the remote.
		const hrt_abstime dt = (local_us > _last_update_us) ? (local_us - _last_update_us) : 0;
		_link_offset_us += static_cast<int64_t>(dt) * kMaxClockDriftPpm / 1'000'000;

		if (lag < _link_offset_us) {
			_link_offset_us = lag;
		}
	}

	_last_update_us = local_us;

	const int64_t result = static_cast<int64_t>(remote_us) + _link_offset_us;

	// Mapped HRT cannot lie after receipt.
	if (result <= 0 || result > static_cast<int64_t>(local_us)) {
		return 0;
	}

	return static_cast<hrt_abstime>(result);
}
