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
#include <nuttx/irq.h>

namespace uavcan_hrt_clock
{

struct SyncStatus {
	uint32_t adjustments;		///< relative adjustments applied since boot
	int32_t last_adjustment_usec;
	int32_t rate_ppb;		///< rate correction currently applied to HRT
};

/*
 * The libuavcan UTC clock as a line through HRT:
 *
 *     utc = hrt + offset + (hrt - anchor) * rate
 *
 * Each sync adjustment steps the offset by the measured phase error and
 * integrates half of the rate error it implies, so once converged the error
 * left between syncs is the measurement noise rather than a second of crystal
 * drift. The rate is a Q32 fraction so the interrupt path needs one 64-bit
 * multiply and no division.
 *
 * Writers hold a critical section. The CAN interrupt reads without one, which
 * is consistent on a single core because the writer cannot be preempted by it.
 */
class Clock
{
public:
	uint64_t utcUsecFromInterrupt() const
	{
		return _set ? utcAt(hrt_absolute_time()) : 0;
	}

	uint64_t utcUsec() const
	{
		irqstate_t flags = enter_critical_section();
		const uint64_t utc = utcUsecFromInterrupt();
		leave_critical_section(flags);
		return utc;
	}

	void setUtc(uint64_t utc_usec)
	{
		irqstate_t flags = enter_critical_section();
		const uint64_t now = hrt_absolute_time();
		_offset_usec = int64_t(utc_usec) - int64_t(now);
		_anchor = now;
		_last_adjustment_at = now;
		_set = true;
		leave_critical_section(flags);
	}

	/**
	 * Moves UTC by `adjustment_usec`. While UTC is unset the adjustment is
	 * absolute: UTC reads `adjustment_usec` from now on.
	 */
	void adjustUtc(int64_t adjustment_usec)
	{
		irqstate_t flags = enter_critical_section();
		const uint64_t now = hrt_absolute_time();

		if (!_set) {
			_offset_usec = adjustment_usec - int64_t(now);
			_anchor = now;
			_rate_q32 = 0;
			_last_adjustment_at = now;
			_set = true;

		} else {
			reanchor(now);
			_offset_usec += adjustment_usec;

			const int64_t dt = int64_t(now - _last_adjustment_at);
			_last_adjustment_at = now;

			if (adjustment_usec > kStepUsec || adjustment_usec < -kStepUsec) {
				// A jump means a new master or a sync gap; its rate is meaningless.
				_rate_q32 = 0;

			} else if (dt >= kMinRateDtUsec && dt <= kMaxRateDtUsec) {
				_rate_q32 += adjustment_usec * (int64_t(1) << 31) / dt;

				if (_rate_q32 > kMaxRateQ32) { _rate_q32 = kMaxRateQ32; }

				if (_rate_q32 < -kMaxRateQ32) { _rate_q32 = -kMaxRateQ32; }
			}

			_adjustments++;
			_last_adjustment_usec = int32_t(adjustment_usec);
		}

		leave_critical_section(flags);
	}

	SyncStatus status() const
	{
		irqstate_t flags = enter_critical_section();
		const SyncStatus s{_adjustments, _last_adjustment_usec, int32_t((_rate_q32 * 1000000000LL) >> 32)};
		leave_critical_section(flags);
		return s;
	}

private:
	static constexpr int64_t kStepUsec = 10000;
	static constexpr int64_t kMinRateDtUsec = 200000;
	static constexpr int64_t kMaxRateDtUsec = 10000000;
	static constexpr int64_t kMaxRateQ32 = (int64_t(500) << 32) / 1000000;	// 500 ppm

	uint64_t utcAt(uint64_t hrt) const
	{
		return uint64_t(int64_t(hrt) + _offset_usec + ((int64_t(hrt - _anchor) * _rate_q32) >> 32));
	}

	void reanchor(uint64_t now)
	{
		_offset_usec += (int64_t(now - _anchor) * _rate_q32) >> 32;
		_anchor = now;
	}

	bool _set{false};
	int64_t _offset_usec{0};
	int64_t _rate_q32{0};
	uint64_t _anchor{0};
	uint64_t _last_adjustment_at{0};
	uint32_t _adjustments{0};
	int32_t _last_adjustment_usec{0};
};

} // namespace uavcan_hrt_clock
