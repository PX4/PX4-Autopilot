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

/**
 * @file FailureInjection.hpp
 *
 * Shared helper for applying failure injection where a message is produced.
 *
 * The failure_injection manager publishes the deduplicated failure_injection
 * topic. Each producer caches it in a Config (update() per loop), looks up the
 * Mode for its (unit, instance), and applies it: process<MsgT>() covers the
 * generic Off (suppress) / Stuck (replay last sample) for single-message
 * producers, the message-less process() the payload-less Off-only units;
 * value-mutating and multi-instance cases are driver-specific.
 */

#pragma once

#include <cstdint>

#include <uORB/Subscription.hpp>
#include <uORB/topics/failure_injection.h>

struct battery_status_s;

namespace failure_injection
{

enum class Mode : uint8_t {
	Ok           = failure_injection_s::FAILURE_TYPE_OK,
	Off          = failure_injection_s::FAILURE_TYPE_OFF,
	Stuck        = failure_injection_s::FAILURE_TYPE_STUCK,
	Garbage      = failure_injection_s::FAILURE_TYPE_GARBAGE,
	Wrong        = failure_injection_s::FAILURE_TYPE_WRONG,
	Slow         = failure_injection_s::FAILURE_TYPE_SLOW,
	Delayed      = failure_injection_s::FAILURE_TYPE_DELAYED,
	Intermittent = failure_injection_s::FAILURE_TYPE_INTERMITTENT,
};

#if defined(CONFIG_MODULES_FAILURE_INJECTION_MANAGER)

class Config
{
public:
	/**
	 * Poll the failure_injection subscription and, when a new failure_injection
	 * message arrives, refresh the cached failure config that mode() / any_active()
	 * read. Call once per loop iteration.
	 * @return true if an updated failure config was applied, false if unchanged.
	 */
	bool update();

	/** Cache a failure_injection message directly (used by tests). */
	void set(const failure_injection_s &cfg);

	/**
	 * Active failure mode for a (unit, instance), or Mode::Ok if none.
	 * @param unit     one of failure_injection_s::FAILURE_UNIT_*
	 * @param instance 1-based instance, or 0 to match any instance of the unit
	 */
	Mode mode(uint8_t unit, uint8_t instance) const;

	/** True if any failure is currently active (free no-failure early-out). */
	bool any_active() const { return _count > 0; }

private:
	uORB::Subscription _sub{ORB_ID(failure_injection)};

	uint8_t  _count{0};
	uint8_t  _unit[failure_injection_s::MAX_FAILURES] {};
	uint16_t _instance_mask[failure_injection_s::MAX_FAILURES] {};
	Mode     _failure_type[failure_injection_s::MAX_FAILURES] {};
};

/**
 * Per-(consumer, instance) state for stateful failures. Holds the last good
 * sample so Stuck can replay it.
 */
template<typename MsgT>
struct Stuck {
	bool valid{false};
	MsgT value{};
};

template<typename...>
using void_t = void;

template<typename T, typename = void>
struct has_timestamp_sample {
	static constexpr bool value = false;
};
template<typename T>
struct has_timestamp_sample<T, void_t<decltype(T::timestamp_sample)>> {
	static constexpr bool value = true;
};

/**
 * Generic whole-message processor for the Ok / Off / Stuck mechanics, for consumers
 * that publish a single message and want the message-agnostic behaviour.
 *
 * @return false if the message must be suppressed (Off); true otherwise. For
 *         Stuck the message is overwritten with the last good sample.
 */
template<typename MsgT>
bool process(Mode mode, MsgT &msg, Stuck<MsgT> &stuck)
{
	switch (mode) {
	case Mode::Off:
		return false;

	case Mode::Stuck:
		if (stuck.valid) {
			const uint64_t timestamp = msg.timestamp;

			if constexpr(has_timestamp_sample<MsgT>::value) {
				const uint64_t timestamp_sample = msg.timestamp_sample;
				msg = stuck.value;
				msg.timestamp_sample = timestamp_sample;

			} else {
				msg = stuck.value;
			}

			msg.timestamp = timestamp;
		}

		return true;

	default:
		stuck.value = msg;
		stuck.valid = true;
		return true;
	}
}

/**
 * Convenience overload to fix the fact that the uORB instance is 0-based but the failure_injection instance is 1-based.
 */
template<typename MsgT>
bool process(const Config &config, uint8_t unit, uint8_t uorb_instance, MsgT &msg, Stuck<MsgT> &stuck)
{
	return process(config.mode(unit, uorb_instance + 1), msg, stuck);
}

/**
 * Message-less variant for producers with no payload to replay (e.g. a heartbeat)
 *
 * @return false if the signal must be treated as failed/suppressed (Off), true otherwise.
 */
inline bool process(Mode mode)
{
	return mode != Mode::Off;
}

/** Convenience overload, same 0-based to 1-based instance mapping as the generic process(). */
inline bool process(const Config &config, uint8_t unit, uint8_t uorb_instance)
{
	return process(config.mode(unit, uorb_instance + 1));
}

/**
 * Battery counterpart to process(): on FAILURE_UNIT_SYSTEM_BATTERY Off for the given 1-based
 * instance, report a depleted pack (zero remaining, emergency warning) so the low-battery
 * failsafe triggers. Mutates rather than suppresses, so the pack reads empty not disconnected.
 */
void process_battery(const Config &config, uint8_t instance, battery_status_s &battery_status);

#else // !CONFIG_MODULES_FAILURE_INJECTION_MANAGER

class Config
{
public:
	bool update() { return false; }
	void set(const failure_injection_s &) {}
	Mode mode(uint8_t, uint8_t) const { return Mode::Ok; }
	bool any_active() const { return false; }
};

template<typename MsgT>
struct Stuck {
};

template<typename MsgT>
bool process(Mode, MsgT &, Stuck<MsgT> &) { return true; }

template<typename MsgT>
bool process(const Config &, uint8_t, uint8_t, MsgT &, Stuck<MsgT> &) { return true; }

inline bool process(Mode) { return true; }

inline bool process(const Config &, uint8_t, uint8_t) { return true; }

inline void process_battery(const Config &, uint8_t, battery_status_s &) {}

#endif // CONFIG_MODULES_FAILURE_INJECTION_MANAGER

} // namespace failure_injection
