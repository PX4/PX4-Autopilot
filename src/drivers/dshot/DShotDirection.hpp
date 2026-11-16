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
#pragma once

#include <stdint.h>

/**
 * Sequencing for temporary ESC direction commands.
 *
 * The ESC gives no acknowledgement, and a burst it misses is silent: PX4 would report the new
 * direction while the motors keep the old one. The direction commands only set a state, so the
 * whole burst is sent several times, separated by stop frames, and any one received burst is enough.
 */
class DShotDirection
{
public:
	enum class Phase { Idle, Stop, Command, Gap, Settle, Complete };
	static constexpr uint64_t StopTime = 1000000; // microseconds
	static constexpr uint64_t SettleTime = 200000;
	static constexpr uint64_t CommandInterval = 1000;
	static constexpr uint8_t CommandRepetitions = 10;
	static constexpr uint8_t CommandBursts = 5;
	static constexpr uint64_t BurstGap = 150000; // stop frames between bursts

	void observeArming(uint64_t timestamp, bool armed)
	{
		if (!armed && timestamp > _timestamp_disarmed) {
			_timestamp_disarmed = timestamp;
			_session = Session::Available;
		}

		_armed = armed;
	}

	bool canRequest() const { return _armed && _session == Session::Available && !active(); }
	uint64_t timestampDisarmed() const { return _timestamp_disarmed; }

	bool requestReverse(uint64_t now, uint64_t timestamp, uint64_t timestamp_sample, uint32_t id, uint16_t mask)
	{
		static constexpr uint64_t RequestTimeout = 500000; // microseconds

		if (!canRequest() || id == 0 || mask == 0 || _timestamp_disarmed == 0 || timestamp_sample <= _timestamp_disarmed
		    || timestamp_sample > timestamp || timestamp > now || now - timestamp_sample >= RequestTimeout) {
			return false;
		}

		_session = Session::Accepted;
		_request_id = id;
		_mask = mask;
		_restoring = false;
		startStop(now);
		return true;
	}

	// Called whenever the driver cannot drive, including while still armed.
	bool cancel(uint64_t now)
	{
		if (_armed) {
			if (_session == Session::Accepted) {
				_session = Session::Cancelled;

			} else if (_session == Session::Available) {
				_session = Session::Blocked;
			}
		}

		if (!active() || _restoring) {
			return false;
		}

		if (_may_be_reversed) {
			_restoring = true;
			startStop(now);

		} else {
			_phase = Phase::Complete;
		}

		return true;
	}

	void update(uint64_t now)
	{
		if (_phase == Phase::Stop && now - _phase_started >= StopTime) {
			_bursts_remaining = CommandBursts;
			startBurst();

		} else if (_phase == Phase::Gap && now - _phase_started >= BurstGap) {
			startBurst();

		} else if (_phase == Phase::Settle && now - _phase_started >= SettleTime) {
			if (_restoring) {
				_may_be_reversed = false;
			}

			_phase = Phase::Complete;
		}
	}

	bool commandDue(uint64_t now) const
	{
		return _phase == Phase::Command && (_last_command == 0 || now - _last_command >= CommandInterval);
	}

	void commandAttempted(uint64_t now, uint16_t submitted_motor_mask)
	{
		_last_command = now;
		const uint16_t submitted_targets = submitted_motor_mask & _mask;

		if (submitted_targets == 0) {
			return;
		}

		if (!_restoring) {
			// Once any target may have reversed, keep the entire group covered by restoration.
			_may_be_reversed = true;
		}

		if (submitted_targets == _mask && --_commands_remaining == 0) {
			_phase = --_bursts_remaining > 0 ? Phase::Gap : Phase::Settle;
			_phase_started = now;
		}
	}

	bool pending() const { return _phase != Phase::Idle && _phase != Phase::Complete; }
	bool active() const { return pending() || _may_be_reversed; }
	bool throttleInhibited() const { return _session == Session::Cancelled; }
	Phase phase() const { return _phase; }
	uint32_t requestId() const { return _request_id; }
	uint16_t mask() const { return _mask; }
	uint16_t reversedMask() const { return _may_be_reversed ? _mask : 0; }
	bool restoring() const { return _restoring; }

private:
	enum class Session { Available, Accepted, Blocked, Cancelled };

	void startStop(uint64_t now)
	{
		_phase = Phase::Stop;
		_phase_started = now;
	}

	void startBurst()
	{
		_phase = Phase::Command;
		_last_command = 0;
		_commands_remaining = CommandRepetitions;
	}

	Phase _phase{Phase::Idle};
	Session _session{Session::Available};
	uint64_t _phase_started{0};
	uint64_t _last_command{0};
	uint64_t _timestamp_disarmed{0};
	uint32_t _request_id{0};
	uint16_t _mask{0};
	uint8_t _commands_remaining{0};
	uint8_t _bursts_remaining{0};
	bool _armed{false};
	bool _restoring{false};
	bool _may_be_reversed{false};
};
