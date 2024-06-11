/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "MulticopterThrowLaunch.hpp"
#include <px4_platform_common/events.h>

MulticopterThrowLaunch::MulticopterThrowLaunch(ModuleParams *parent) :
	ModuleParams(parent)
{}

void MulticopterThrowLaunch::update(const bool armed)
{
	if (_param_com_throw_en.get()) {
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position{};

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_last_velocity = matrix::Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
			}
		}

		if (!armed && _throw_launch_state != ThrowLaunchState::IDLE) {
			events::send(events::ID("mc_throw_launch_not_ready"), events::Log::Critical, "Disarmed, don't throw");
			_throw_launch_state = ThrowLaunchState::IDLE;
		}

		switch (_throw_launch_state) {
		case ThrowLaunchState::IDLE:
			if (armed) {
				events::send(events::ID("mc_throw_launch_ready"), events::Log::Critical, "Ready for throw launch");
				_throw_launch_state = ThrowLaunchState::ARMED;
			}

			break;

		case ThrowLaunchState::ARMED:
			if (_last_velocity.longerThan(_param_com_throw_min_speed.get())) {
				PX4_INFO("Throw detected, motors will start once falling");
				_throw_launch_state = ThrowLaunchState::UNSAFE;
			}

			break;

		case ThrowLaunchState::UNSAFE:
			if (_last_velocity(2) > 0.f) {
				PX4_INFO("Throw and fall detected, starting motors");
				_throw_launch_state = ThrowLaunchState::FLYING;
			}

			break;

		case ThrowLaunchState::DISABLED:
		case ThrowLaunchState::FLYING:
			// Nothing to do
			break;
		}

	} else if (_throw_launch_state != ThrowLaunchState::DISABLED) {
		// make sure everything is reset when the throw launch is disabled
		_throw_launch_state = ThrowLaunchState::DISABLED;
	}
}
