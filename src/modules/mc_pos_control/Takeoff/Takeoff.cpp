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
 * @file Takeoff.cpp
 */

#include "Takeoff.hpp"
#include <mathlib/mathlib.h>

struct StateParams
{
	StateParams(Takeoff *parent, const bool armed, const bool landed, const bool want_takeoff,
				 const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
		: _parent(parent),
		  _armed(armed), 
		  _landed(landed), 
		  _want_takeoff(want_takeoff), 
		  _skip_takeoff(skip_takeoff), 
		  _takeoff_desired_vz(takeoff_desired_vz), 
		  _now_us(now_us)
	{
	}

	Takeoff *_parent;
	bool _armed, _landed, _want_takeoff, _skip_takeoff;
	float _takeoff_desired_vz;
	const hrt_abstime &_now_us;
};

class Takeoff::StateHandler
{
public:
	static void handleDisarmed(const StateParams& params);
	static void handleSpoolUp(const StateParams& params);
	static void handleReadyForTakeoff(const StateParams& params);
	static void handleRampUp(const StateParams& params);
	static void handleFlight(const StateParams& params);
};

void Takeoff::StateHandler::handleDisarmed(const StateParams& params)
{
	if (params._armed)
	{
		params._parent->_takeoff_state = TakeoffState::spoolup;
		handleSpoolUp(params);
	}
}

void Takeoff::StateHandler::handleSpoolUp(const StateParams& params)
{
	if (params._parent->_spoolup_time_hysteresis.get_state()) 
	{
		params._parent->_takeoff_state = TakeoffState::ready_for_takeoff;
		handleReadyForTakeoff(params);
	}
}

void Takeoff::StateHandler::handleReadyForTakeoff(const StateParams& params)
{
	if (params._want_takeoff) {
		params._parent->_takeoff_state = TakeoffState::rampup;
		params._parent->_takeoff_ramp_vz = params._parent->_takeoff_ramp_vz_init;
		handleRampUp(params);
	}
}

void Takeoff::StateHandler::handleRampUp(const StateParams& params)
{
	if (params._parent->_takeoff_ramp_vz >= params._takeoff_desired_vz) 
	{
		params._parent->_takeoff_state = TakeoffState::flight;
		handleFlight(params);
	}
}

void Takeoff::StateHandler::handleFlight(const StateParams& params)
{
	if (params._landed) {
		params._parent->_takeoff_state = TakeoffState::ready_for_takeoff;
	}
}

void Takeoff::generateInitialRampValue(const float hover_thrust, float velocity_p_gain)
{
	velocity_p_gain = math::max(velocity_p_gain, 0.01f);
	_takeoff_ramp_vz_init = -hover_thrust / velocity_p_gain;
}

void Takeoff::updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				 const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
{
	_spoolup_time_hysteresis.set_state_and_update(armed, now_us);

	if (!armed)	{
		_takeoff_state = TakeoffState::disarmed;
	}
	else if (armed && skip_takeoff)	{
		_takeoff_state = TakeoffState::flight;
	}
	else
	{
		StateParams params{this, armed, landed, want_takeoff, takeoff_desired_vz, skip_takeoff, now_us};

		switch (_takeoff_state) {
			case TakeoffState::disarmed:
				StateHandler::handleDisarmed(params);
				break;
			case TakeoffState::spoolup:
				StateHandler::handleSpoolUp(params);
				break;
			case TakeoffState::ready_for_takeoff:
				StateHandler::handleReadyForTakeoff(params);
				break;
			case TakeoffState::rampup:
				StateHandler::handleRampUp(params);
				break;
			case TakeoffState::flight:
				StateHandler::handleFlight(params);
				break;
			default:
				break;
		}
	}
}

float Takeoff::updateRamp(const float dt, const float takeoff_desired_vz)
{
	if (_takeoff_state < TakeoffState::rampup) {
		return _takeoff_ramp_vz_init;
	}

	if (_takeoff_state == TakeoffState::rampup) {
		if (_takeoff_ramp_time > dt) {
			_takeoff_ramp_vz += (takeoff_desired_vz - _takeoff_ramp_vz_init) * dt / _takeoff_ramp_time;

		} else {
			_takeoff_ramp_vz = takeoff_desired_vz;
		}

		if (_takeoff_ramp_vz < takeoff_desired_vz) {
			return _takeoff_ramp_vz;
		}
	}

	return takeoff_desired_vz;
}
