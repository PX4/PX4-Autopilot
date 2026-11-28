/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file SubscriptionBlocking.cpp
 *
 * Out-of-line, type-independent part of SubscriptionBlocking.
 */

#include "SubscriptionBlocking.hpp"

#include <containers/LockGuard.hpp>
#include <px4_time.h>

namespace uORB
{

SubscriptionBlockingBase::SubscriptionBlockingBase(const orb_metadata *meta, uint32_t interval_us, uint8_t instance) :
	SubscriptionCallback(meta, interval_us, instance)
{
}

SubscriptionBlockingBase::~SubscriptionBlockingBase()
{
	unregisterCallback();
	pthread_mutex_destroy(&_mutex);
	pthread_cond_destroy(&_cv);
}

void SubscriptionBlockingBase::call(unsigned generation)
{
	pthread_cond_signal(&_cv);
}

bool SubscriptionBlockingBase::updatedBlocking(uint32_t timeout_us)
{
	if (!registered()) {
		registerCallback();
	}

	if (updated()) {
		// return immediately if updated
		return true;

	} else {
		// otherwise wait

		LockGuard lg{_mutex};

		if (timeout_us == 0) {
			// wait with no timeout
			if (pthread_cond_wait(&_cv, &_mutex) == 0) {
				return updated();
			}

		} else {
			// otherwise wait with timeout based on interval

			// Calculate an absolute time in the future
			struct timespec ts;
			px4_clock_gettime(CLOCK_REALTIME, &ts);
			uint64_t nsecs = ts.tv_nsec + (timeout_us * 1000);
			static constexpr unsigned billion = (1000 * 1000 * 1000);
			ts.tv_sec += nsecs / billion;
			nsecs -= (nsecs / billion) * billion;
			ts.tv_nsec = nsecs;

			if (px4_pthread_cond_timedwait(&_cv, &_mutex, &ts) == 0) {
				return updated();
			}
		}
	}

	return false;
}

} // namespace uORB
