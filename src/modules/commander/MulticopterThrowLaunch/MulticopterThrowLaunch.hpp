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

/**
 * @file MulticopterThrowLaunch.hpp
 *
 * Changes to manage a takeoff of a multicopter by manually throwing it into the air.
 *
 * @author Michał Barciś <mbarcis@mbarcis.net>
 */

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

class MulticopterThrowLaunch : public ModuleParams
{
public:
	explicit MulticopterThrowLaunch(ModuleParams *parent);
	~MulticopterThrowLaunch() override = default;

	/**
	 * @return false if feature disabled or already flying
	 */
	bool isThrowLaunchInProgress() const
	{
		return _throw_launch_state != ThrowLaunchState::DISABLED
		       && _throw_launch_state != ThrowLaunchState::FLYING;
	}

	bool isReadyToThrow() const { return _throw_launch_state == ThrowLaunchState::ARMED; }

	/**
	 * Main update of the state
	 * @param armed true if vehicle is armed
	 */
	void update(const bool armed);

	enum class ThrowLaunchState {
		DISABLED = 0,
		IDLE = 1,
		ARMED = 2,
		UNSAFE = 3,
		FLYING = 4
	};

private:
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	ThrowLaunchState _throw_launch_state{ThrowLaunchState::DISABLED};
	matrix::Vector3f _last_velocity{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::COM_THROW_EN>) _param_com_throw_en,
		(ParamFloat<px4::params::COM_THROW_SPEED>) _param_com_throw_min_speed
	);
};
