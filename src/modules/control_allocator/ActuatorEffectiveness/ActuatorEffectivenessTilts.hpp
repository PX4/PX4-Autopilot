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

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

#include <px4_platform_common/module_params.h>

class ActuatorEffectivenessTilts : public ModuleParams, public ActuatorEffectiveness
{
public:

	static constexpr int MAX_COUNT = 4;

	enum class Control : int32_t {
		// This matches with the parameter
		None = 0,
		Yaw = 1,
		Pitch = 2,
		YawAndPitch = 3,
	};
	enum class TiltDirection : int32_t {
		// This matches with the parameter
		TowardsFront = 0,
		TowardsRight = 90,
	};

	struct Params {
		Control control;
		float min_angle;
		float max_angle;
		TiltDirection tilt_direction;
	};

	ActuatorEffectivenessTilts(ModuleParams *parent);
	virtual ~ActuatorEffectivenessTilts() = default;

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "Tilts"; }

	int count() const { return _count; }

	const Params &config(int idx) const { return _params[idx]; }

	void updateTorqueSign(const ActuatorEffectivenessRotors::Geometry &geometry, bool disable_pitch = false);

	bool hasYawControl() const;

	float getYawTorqueOfTilt(int tilt_index) const { return _torque[tilt_index](2); }

private:
	void updateParams() override;

	struct ParamHandles {
		param_t control;
		param_t min_angle;
		param_t max_angle;
		param_t tilt_direction;
	};

	ParamHandles _param_handles[MAX_COUNT];
	param_t _count_handle;

	Params _params[MAX_COUNT] {};
	int _count{0};

	matrix::Vector3f _torque[MAX_COUNT] {};
};
