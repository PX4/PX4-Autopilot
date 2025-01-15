/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <lib/slew_rate/SlewRate.hpp>

static constexpr float kFlapSlewRate = 0.5f; // slew rate for normalized flaps setpoint [1/s]
static constexpr float kSpoilersSlewRate = 0.5f; // slew rate for normalized spoilers setpoint [1/s]

class ActuatorEffectivenessControlSurfaces : public ModuleParams, public ActuatorEffectiveness
{
public:

	static constexpr int MAX_COUNT = 8;

	enum class Type : int32_t {
		// This matches with the parameter
		LeftAileron = 1,
		RightAileron = 2,
		Elevator = 3,
		Rudder = 4,
		LeftElevon = 5,
		RightElevon = 6,
		LeftVTail = 7,
		RightVTail = 8,
		LeftFlap = 9,
		RightFlap = 10,
		Airbrake = 11,
		Custom = 12,
		LeftATail = 13,
		RightATail = 14,
		SingleChannelAileron = 15,
		SteeringWheel = 16,
		LeftSpoiler = 17,
		RightSpoiler = 18,
	};

	struct Params {
		Type type;

		matrix::Vector3f torque;
		float trim;
		float scale_flap;
		float scale_spoiler;
	};

	ActuatorEffectivenessControlSurfaces(ModuleParams *parent);
	virtual ~ActuatorEffectivenessControlSurfaces() = default;

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "Control Surfaces"; }

	int count() const { return _count; }

	const Params &config(int idx) const { return _params[idx]; }

	void applyFlaps(float flaps_control, int first_actuator_idx, float dt, ActuatorVector &actuator_sp);
	void applySpoilers(float spoilers_control, int first_actuator_idx, float dt, ActuatorVector &actuator_sp);

private:
	void updateParams() override;

	struct ParamHandles {
		param_t type;

		param_t torque[3];
		param_t trim;
		param_t scale_flap;
		param_t scale_spoiler;
	};
	ParamHandles _param_handles[MAX_COUNT];
	param_t _count_handle;

	Params _params[MAX_COUNT] {};
	int _count{0};

	SlewRate<float> _flaps_setpoint_with_slewrate;
	SlewRate<float> _spoilers_setpoint_with_slewrate;
};
