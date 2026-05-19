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

/**
 * Maps a GNSS PVT UTC timestamp to the FC's HRT timebase so that the EKF can
 * align GPS samples with IMU samples without relying on the SENS_GPS*_DELAY
 * constant-latency guess.
 *
 * Each observation gives cand = UTC_PVT - HRT_recv = (UTC - HRT) - latency,
 * where latency >= 0. The true offset (UTC - HRT) is therefore the supremum
 * of cand over recent observations. Estimated by an asymmetric leaky-max:
 *
 *   - snap up whenever cand > current_offset (tracks lower latency and any
 *     UTC-faster-than-HRT drift),
 *   - decay slowly between observations at the worst-case FC crystal drift
 *     rate so the offset can also follow UTC-slower-than-HRT drift without
 *     accumulating bias over long flights.
 *
 * Residual bias of the mapped timestamp is approximately min(latency) over
 * the recent window -- a few ms at most on typical CAN/serial GPS, three
 * orders of magnitude better than the 110 ms SENS_GPS_DELAY fallback.
 *
 * Output is clamped to be strictly less than hrt_recv_us (so downstream
 * "did the driver set it" checks don't misfire) and strictly greater than
 * the previously emitted value (so the EKF GPS-buffer push doesn't reject
 * non-monotonic samples during the convergence transient).
 */
class UtcToHrtMapper
{
public:
	UtcToHrtMapper() = default;
	~UtcToHrtMapper() = default;

	/**
	 * @param utc_us       PVT UTC timestamp [us since Unix epoch]; 0 if unavailable.
	 * @param hrt_recv_us  HRT timestamp captured when the GNSS message was received.
	 * @return mapped HRT timestamp, or 0 if mapping not available for this sample.
	 */
	hrt_abstime map(uint64_t utc_us, hrt_abstime hrt_recv_us);

private:
	// Bound on FC HSE crystal drift vs GNSS atomic time. 100 ppm safely envelopes
	// raw crystals over temperature; TCXOs are at most a few ppm.
	static constexpr int64_t kMaxClockDriftPpm{100};

	// A negative jump in cand larger than this can't be FC drift -- treat as a
	// UTC discontinuity (leap-second insertion, receiver recovery) and reset.
	static constexpr int64_t kUtcDiscontinuityResetUs{100'000};

	int64_t     _offset_us{0};       // estimate of UTC - HRT
	hrt_abstime _last_update_us{0};
	hrt_abstime _last_emitted_us{0};
	bool        _valid{false};
};
