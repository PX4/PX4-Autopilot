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
 * @file SubscriptionInterval.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

#include "Subscription.hpp"

#include <mathlib/mathlib.h>

namespace uORB
{

namespace subscription_interval_detail
{

// Drop-in, non-atomic stand-in for px4::atomic. It mirrors the load()/store()
// interface so the shared SubscriptionIntervalBase body works unchanged, but
// compiles down to plain member access with no synchronization overhead.
template <typename T>
class NonAtomic
{
public:
	NonAtomic() = default;
	explicit NonAtomic(T value) : _value(value) {}
	T load() const { return _value; }
	void store(T value) { _value = value; }
private:
	T _value{};
};

// Minimal compile-time type selector (std::conditional is unavailable: the STL is
// not used in firmware headers that also build for NuttX).
template <bool ATOMIC, typename T> struct storage { using type = px4::atomic<T>; };
template <typename T> struct storage<false, T> { using type = NonAtomic<T>; };

template <bool ATOMIC, typename T>
using storage_t = typename storage<ATOMIC, T>::type;

} // namespace subscription_interval_detail

// Base subscription wrapper class
//
// ATOMIC selects whether the interval and last-update fields use atomic storage.
// They only need to be atomic for callback subscriptions (SubscriptionCallback and
// derived), where call() runs on the publishing thread while the subscriber thread
// updates them. Plain subscriptions are single-threaded, so the default keeps the
// fields as plain members. Use the SubscriptionInterval / SubscriptionIntervalAtomic
// aliases below rather than this template directly.
template <bool ATOMIC = false>
class SubscriptionIntervalBase
{
public:

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 * @param interval The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionIntervalBase(ORB_ID id, uint32_t interval_us = 0, uint8_t instance = 0) :
		_subscription{id, instance},
		_interval_us(interval_us)
	{}

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionIntervalBase(const orb_metadata *meta, uint32_t interval_us = 0, uint8_t instance = 0) :
		_subscription{meta, instance},
		_interval_us(interval_us)
	{}

	SubscriptionIntervalBase() : _subscription{nullptr} {}

	~SubscriptionIntervalBase() = default;

	bool subscribe() { return _subscription.subscribe(); }
	void unsubscribe() { _subscription.unsubscribe(); }

	bool advertised() { return _subscription.advertised(); }

	/**
	 * Check if there is a new update.
	 * */
	bool updated();

	/**
	 * Copy the struct if updated.
	 * @param dst The destination pointer where the struct will be copied.
	 * @return true only if topic was updated and copied successfully.
	 */
	bool update(void *dst);

	/**
	 * Copy the struct
	 * @param dst The destination pointer where the struct will be copied.
	 * @return true only if topic was copied successfully.
	 */
	bool copy(void *dst);

	bool		valid() const { return _subscription.valid(); }

	uint8_t		get_instance() const { return _subscription.get_instance(); }
	uint32_t        get_interval_us() const { return _interval_us.load(); }
	unsigned	get_last_generation() const { return _subscription.get_last_generation(); }
	orb_id_t	get_topic() const { return _subscription.get_topic(); }

	/**
	 * Set the interval in microseconds
	 * @param interval The interval in microseconds.
	 */
	void		set_interval_us(uint32_t interval) { _interval_us.store(interval); }

	/**
	 * Set the interval in milliseconds
	 * @param interval The interval in milliseconds.
	 */
	void		set_interval_ms(uint32_t interval) { _interval_us.store(interval * 1000); }

	/**
	 * Set the last data update
	 * @param t should be in range [now, now - _interval_us]
	 */
	void		set_last_update(hrt_abstime t) { _last_update.store(t); }
protected:

	Subscription	_subscription;
	subscription_interval_detail::storage_t<ATOMIC, uint64_t>	_last_update{0};	// last subscription update in microseconds
	subscription_interval_detail::storage_t<ATOMIC, uint32_t>	_interval_us{0};	// maximum update interval in us

};

// Single-threaded subscription with a minimum update interval (the common case).
using SubscriptionInterval = SubscriptionIntervalBase<false>;

// Same, but with atomic interval / last-update fields for the callback subscriptions,
// where call() runs on the publishing thread concurrently with the subscriber.
using SubscriptionIntervalAtomic = SubscriptionIntervalBase<true>;

// The out-of-line methods are explicitly instantiated in SubscriptionInterval.cpp.
// Declaring them extern here keeps other translation units from emitting their own
// weak copies, so the single strong definition can be pinned into ITCM by name on
// the boards that do so (see boards/*/nuttx-config/scripts/itcm_*.ld).
extern template bool SubscriptionIntervalBase<false>::updated();
extern template bool SubscriptionIntervalBase<false>::update(void *dst);
extern template bool SubscriptionIntervalBase<false>::copy(void *dst);
extern template bool SubscriptionIntervalBase<true>::updated();
extern template bool SubscriptionIntervalBase<true>::update(void *dst);
extern template bool SubscriptionIntervalBase<true>::copy(void *dst);

} // namespace uORB
