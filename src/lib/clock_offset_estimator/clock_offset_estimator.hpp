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
#include <drivers/drv_hrt.h>

// Estimates the offset between a remote node's clock and the FC HRT timebase
// with a leaky-min filter on (HRT_recv - remote_ts), then maps remote
// timestamps into HRT. Residual bias is the minimum observed transport delay;
// the offset drifts upward between observations to track clock skew, and
// hard-resets on jumps larger than any plausible transport stall.
class ClockOffsetEstimator
{
public:
	ClockOffsetEstimator() = default;
	~ClockOffsetEstimator() = default;

	// Returns the HRT estimate of the remote event, or 0 if unavailable.
	// remote_us: remote timestamp [us], same epoch across calls; 0 if unknown.
	// local_us:  HRT when the carrying message arrived.
	hrt_abstime toLocal(uint64_t remote_us, hrt_abstime local_us);

private:
	// 100 ppm envelopes raw HSE crystals over temperature; TCXOs are a few ppm.
	static constexpr int64_t kMaxClockDriftPpm{100};

	// Upward lag jumps beyond this are treated as remote-clock discontinuities.
	static constexpr int64_t kResetThresholdUs{500'000};

	int64_t     _link_offset_us{0};
	hrt_abstime _last_update_us{0};
	bool        _initialized{false};
};
