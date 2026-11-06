/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file CanForwardSession.hpp
 *
 * State of a MAVLink CAN forwarding session (MAV_CMD_CAN_FORWARD).
 *
 * There is one session per vehicle. The client keeps it alive by resending MAV_CMD_CAN_FORWARD
 * about once a second; without a keepalive it ends after KeepaliveTimeout. A request from a
 * different client takes the session over.
 *
 * Frames from the GCS are only put on the bus while the vehicle is neither armed nor prearmed
 * ("blocked"). While blocked an existing session keeps forwarding bus traffic to the GCS
 * (MonitorOnly), but no new session can be started.
 *
 * Time is passed in so the class can be tested without a clock.
 */

#pragma once

#include <stdint.h>

namespace can_forward
{

class CanForwardSession
{
public:
	static constexpr uint64_t KeepaliveTimeout = 5'000'000; ///< [us] same as ArduPilot

	enum class State : uint8_t {
		Idle = 0,
		Active = 1,      ///< bus -> GCS and GCS -> bus
		MonitorOnly = 2, ///< bus -> GCS only, vehicle armed or prearmed
	};

	enum class Result : uint8_t {
		Accepted,
		Denied,              ///< invalid bus
		TemporarilyRejected, ///< vehicle armed or prearmed
	};

	/**
	 * Handle MAV_CMD_CAN_FORWARD.
	 *
	 * @param param1    bus, 1-based; 0 stops forwarding
	 * @param num_buses number of CAN interfaces
	 * @param blocked   vehicle is armed or prearmed
	 */
	Result request(float param1, uint8_t num_buses, bool blocked, uint64_t now, uint32_t sysid, uint8_t compid);

	/**
	 * Apply timeout and arming changes. Call periodically, independent of bus traffic.
	 * @return true if the state changed
	 */
	bool update(bool blocked, uint64_t now);

	State state() const { return _state; }
	bool active() const { return _state != State::Idle; }

	/// 0-based bus index, only valid while active()
	uint8_t bus() const { return _bus; }

	uint32_t sysid() const { return _sysid; }
	uint8_t compid() const { return _compid; }

	/// frames received on this bus are forwarded to the GCS
	bool allowForward(uint8_t bus) const { return _state != State::Idle && bus == _bus; }

	/// frames from the GCS may be sent on this bus
	bool allowInject(uint8_t bus) const { return _state == State::Active && bus == _bus; }

private:
	State _state{State::Idle};
	uint8_t _bus{0};
	uint32_t _sysid{0};
	uint8_t _compid{0};
	uint64_t _last_keepalive{0};
};

} // namespace can_forward
