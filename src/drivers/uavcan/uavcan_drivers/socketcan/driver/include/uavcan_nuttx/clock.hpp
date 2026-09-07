/****************************************************************************
 *
 *   Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *   NuttX SocketCAN port Copyright (C) 2022 NXP Semiconductors
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

#include <cstdint>
#include <time.h>

#include <uavcan/driver/system_clock.hpp>

namespace uavcan_socketcan
{
namespace clock
{
/**
 * Performs UTC phase adjustment on the shared clock instance.
 * The UTC time is zero until the first adjustment has been performed.
 */
void adjustUtc(uavcan::UtcDuration adjustment);
}

/**
 * Monotonic time is CLOCK_MONOTONIC, the clock behind SO_TIMESTAMP on
 * received frames. UTC is private to this driver: the first adjustment sets
 * it (PX4 seeds it with hrt_absolute_time() so the bus time base is the FC's
 * HRT), later ones move it by the given offset.
 */
class SystemClock : public uavcan::ISystemClock
{
	uavcan::UtcDuration utc_offset_;
	bool utc_set_{false};
	std::uint64_t adj_cnt_{0};

	static constexpr std::uint64_t UInt1e6 = 1000000;

public:
	uavcan::MonotonicTime getMonotonic() const override
	{
		timespec ts{};
		(void)clock_gettime(CLOCK_MONOTONIC, &ts);
		return uavcan::MonotonicTime::fromUSec(std::uint64_t(ts.tv_sec) * UInt1e6 + ts.tv_nsec / 1000);
	}

	uavcan::UtcTime getUtc() const override
	{
		return utcFromMonotonic(getMonotonic());
	}

	/**
	 * UTC corresponding to a monotonic timestamp, e.g. a frame's SO_TIMESTAMP.
	 * Zero while UTC is unset.
	 */
	uavcan::UtcTime utcFromMonotonic(uavcan::MonotonicTime mono) const
	{
		if (!utc_set_) {
			return uavcan::UtcTime();
		}

		return uavcan::UtcTime::fromUSec(mono.toUSec()) + utc_offset_;
	}

	void adjustUtc(const uavcan::UtcDuration adjustment) override
	{
		if (!utc_set_) {
			// First adjustment is absolute: UTC now equals the adjustment
			utc_offset_ = adjustment - uavcan::UtcDuration::fromUSec(getMonotonic().toUSec());
			utc_set_ = true;

		} else {
			utc_offset_ += adjustment;
		}

		adj_cnt_++;
	}

	bool isUtcSet() const { return utc_set_; }

	std::uint64_t getAdjustmentCount() const { return adj_cnt_; }

	static SystemClock &instance()
	{
		static SystemClock self;
		return self;
	}
};

}
