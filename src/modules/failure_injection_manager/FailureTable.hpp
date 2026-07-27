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
 * @file FailureTable.hpp
 *
 * Policy core of the failure injection manager: the set of currently active
 * failures. Pure logic (no uORB runtime) so it can be unit tested in isolation.
 *
 * The manager feeds each MAV_CMD_INJECT_FAILURE request to inject() and, when the
 * table changes, serialises it into a failure_injection message with fill().
 * inject() reports the ack result the manager should return:
 *   - Accepted    the request was applied (or was a no-op duplicate),
 *   - Unsupported the (unit, type) is not in the capability catalogue,
 *   - Rejected    applying it would exceed MAX_FAILURES distinct (unit, type)s.
 *
 * Addressing is per (unit, type) with a 16-bit instance mask, so a single entry
 * covers any number of instances (e.g. "all motors off" or "motors 1 and 3 off"
 * is one entry). An instance is in at most one failure mode per unit: injecting
 * a new mode on an instance clears it from the unit's other modes.
 */

#pragma once

#include <cstdint>

#include <uORB/topics/failure_injection.h>

namespace failure_injection
{

class FailureTable
{
public:
	enum class AckResult {
		Accepted,
		Unsupported,
		Rejected,
	};

	/**
	 * Apply a MAV_CMD_INJECT_FAILURE request addressed by a single instance.
	 * @param unit     failure_injection_s::FAILURE_UNIT_*
	 * @param type     failure_injection_s::FAILURE_TYPE_*
	 * @param instance 1-based component/sensor instance, or 0 for all instances
	 */
	AckResult inject(uint8_t unit, uint8_t type, uint8_t instance);

	/**
	 * Apply a MAV_CMD_INJECT_FAILURE request addressed by an instance bitmask (bit i = instance i+1),
	 * matching MAV_CMD_INJECT_FAILURE param4. A mask of 0 addresses nothing (accepted no-op).
	 * @param unit failure_injection_s::FAILURE_UNIT_*
	 * @param type failure_injection_s::FAILURE_TYPE_*
	 * @param mask 16-bit instance bitmask
	 */
	AckResult injectMask(uint8_t unit, uint8_t type, uint16_t mask);

	/** True if the table changed since the last clearChanged(). */
	bool changed() const { return _changed; }
	void clearChanged() { _changed = false; }

	/** Serialise the active table into a message (timestamp left to the caller). */
	void fill(failure_injection_s &msg) const;

	uint8_t count() const { return _count; }

	/** Capability catalogue: which (unit, type) combinations the system supports. */
	static bool isSupported(uint8_t unit, uint8_t type);

private:
	struct Entry {
		uint8_t  unit;
		uint16_t instance_mask;
		uint8_t  type;
	};

	static uint16_t instanceToMask(uint8_t instance);
	int findEntry(uint8_t unit, uint8_t type) const;
	void clearInstances(uint8_t unit, uint16_t mask, uint8_t keep_type);
	void compact();

	Entry   _entries[failure_injection_s::MAX_FAILURES] {};
	uint8_t _count{0};
	bool    _changed{false};
};

} // namespace failure_injection
