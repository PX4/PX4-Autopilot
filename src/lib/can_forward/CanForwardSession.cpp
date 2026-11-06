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

#include "CanForwardSession.hpp"

#include <math.h>

namespace can_forward
{

CanForwardSession::Result CanForwardSession::request(float param1, uint8_t num_buses, bool blocked, uint64_t now,
		uint32_t sysid, uint8_t compid)
{
	// the bus is an integer sent as float, anything else is a malformed request
	if (!isfinite(param1) || param1 < 0.f || param1 > 255.f || fabsf(param1 - roundf(param1)) > 1e-3f) {
		return Result::Denied;
	}

	const int bus_1based = static_cast<int>(roundf(param1));

	if (bus_1based == 0) {
		// stopping is always allowed
		_state = State::Idle;
		return Result::Accepted;
	}

	if (bus_1based > num_buses) {
		return Result::Denied;
	}

	const uint8_t bus = static_cast<uint8_t>(bus_1based - 1);
	const bool same_session = active() && bus == _bus && sysid == _sysid && compid == _compid;

	if (blocked && !same_session) {
		// no new session, bus change or takeover while armed or prearmed
		return Result::TemporarilyRejected;
	}

	_bus = bus;
	_sysid = sysid;
	_compid = compid;
	_last_keepalive = now;
	_state = blocked ? State::MonitorOnly : State::Active;
	return Result::Accepted;
}

bool CanForwardSession::update(bool blocked, uint64_t now)
{
	const State previous = _state;

	if (_state != State::Idle) {
		// a clock going backwards must not end the session
		if (now > _last_keepalive && now - _last_keepalive > KeepaliveTimeout) {
			_state = State::Idle;

		} else if (blocked) {
			_state = State::MonitorOnly;

		} else {
			_state = State::Active;
		}
	}

	return _state != previous;
}

} // namespace can_forward
