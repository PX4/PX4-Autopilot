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
 * Shared helper for applying failure injection at the consumer of a message.
 *
 * The failure_injection manager is the sole subscriber to vehicle_command and
 * publishes the deduplicated failure_injection topic. Each consumer caches that
 * topic in a Config and, on its hot path, looks up the active Mode for the
 * (unit, instance) it owns and applies it:
 *
 *   - the no-failure early-out (any_active()) is a cheap local read,
 *   - process<MsgT>() handles the generic Ok / Off / Stuck mechanics
 *     (Off => suppress, Stuck => replay the last good sample),
 *   - composite / value-mutating failures (e.g. GPS Wrong, battery Off,
 *     airspeed ramp) are applied by the consumer based on mode().
 */

#pragma once

#include <cstdint>

#include <uORB/topics/failure_injection.h>

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

/**
 * Cached, queryable view of the failure_injection topic. set() is called when a
 * fresh sample arrives; mode() is a cheap local lookup intended for the hot path.
 */
class Config
{
public:
	/** Cache the latest failure_injection sample. */
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

/**
 * Generic whole-message processor for the Ok / Off / Stuck mechanics.
 *
 * @return false if the message must be suppressed (Off); true otherwise. For
 *         Stuck the message is overwritten with the last good sample (its
 *         timestamp is preserved, so time keeps advancing while the value is
 *         frozen). For every other mode the current sample is recorded as the
 *         last good value and the message is left unchanged.
 *
 * Value-mutating modes (Garbage / Wrong) are intentionally treated as "leave
 * unchanged and record last good" here: their semantics are message-specific
 * and applied by the consumer. Call process() only for modes you are not
 * handling yourself.
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
			msg = stuck.value;
			msg.timestamp = timestamp;
		}

		return true;

	default:
		stuck.value = msg;
		stuck.valid = true;
		return true;
	}
}

} // namespace failure_injection
