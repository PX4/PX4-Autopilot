/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "SubscriptionInterval.hpp"

#include <drivers/drv_hrt.h>

namespace uORB
{

// The bodies are defined out-of-line and explicitly instantiated below (rather
// than inline in the header) to keep them from being emitted at every call site
// - they are shared by many subscribers, so the flash cost matters on target.

template <bool ATOMIC>
bool SubscriptionIntervalBase<ATOMIC>::updated()
{
	hrt_abstime last_update = _last_update.load();

	if (advertised() && (hrt_elapsed_time(&last_update) >= _interval_us.load())) {
		return _subscription.updated();
	}

	return false;
}

template <bool ATOMIC>
bool SubscriptionIntervalBase<ATOMIC>::update(void *dst)
{
	if (updated()) {
		return copy(dst);
	}

	return false;
}

template <bool ATOMIC>
bool SubscriptionIntervalBase<ATOMIC>::copy(void *dst)
{
	if (_subscription.copy(dst)) {
		const hrt_abstime now = hrt_absolute_time();
		const uint32_t interval_us = _interval_us.load();

		// make sure we don't set a timestamp before the timer started counting (now - interval_us would wrap because it's unsigned)
		if (now > interval_us) {
			// shift last update time forward, but don't let it get further behind than the interval
			_last_update.store(math::constrain(_last_update.load() + interval_us, now - interval_us, now));

		} else {
			_last_update.store(now);
		}

		return true;
	}

	return false;
}

// Only these two variants exist, so instantiate them here once each.
template bool SubscriptionIntervalBase<false>::updated();
template bool SubscriptionIntervalBase<false>::update(void *dst);
template bool SubscriptionIntervalBase<false>::copy(void *dst);

template bool SubscriptionIntervalBase<true>::updated();
template bool SubscriptionIntervalBase<true>::update(void *dst);
template bool SubscriptionIntervalBase<true>::copy(void *dst);

} // namespace uORB
