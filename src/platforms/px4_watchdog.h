/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>

namespace px4
{

template<class T>
class Watchdog
{
public:
	Watchdog(unsigned interval, unsigned timeout) :
		_kick_period(perf_alloc(PC_ELAPSED, "WD_kick_period"))
	{
		_interval = interval;
		_timeout = timeout;
	}

	virtual ~Watchdog()
	{
		hrt_cancel(&_call);
		perf_free(_kick_period);
	}

	// Start the watchdog timer with an optional  delay.
	void start(hrt_abstime delay = 0)
	{
		hrt_call_after(&_call, delay, (hrt_callout)&T::watchdog_callback, this);
	}

	// Add additional time to the watchdog timer. This must be called every loop in order to kee the timer from expiring.
	void kick()
	{
		perf_end(_kick_period);

		hrt_abstime new_expiration = hrt_absolute_time() + _interval + _timeout;

		hrt_call_delay(&_call, new_expiration);

		perf_begin(_kick_period);
	}

private:
	// The expiration time scheduled in the future.
	hrt_abstime 		_expiration{};
	struct hrt_call		_call {};
	// The expected interval period of the loop the watchdog is monitoring.
	unsigned 			_interval{};
	// The maximum amount of time allowed to elapse past the specified interval period.
	unsigned 			_timeout{};

	// Perf counters for debugging
	perf_counter_t  _kick_period{};

};

} // namespace px4