/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "ActuatorEffectivenessHelicopter.hpp"

using namespace matrix;

TEST(ActuatorEffectivenessHelicopterTest, ThrottleCurve)
{
	// Disable autosaving parameters to avoid busy loop in param_set()
	param_control_autosave(false);

	// Set parameters for a predefined throttle curve
	float values[5] = {0.f, .3f, .6f, .8f, 1.f};
	param_t param;

	for (int i = 0; i < 5; i++) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		param = param_find(buffer);
		param_set(param, &values[i]);
	}

	ActuatorEffectivenessHelicopter helicopter(nullptr, ActuatorType::MOTORS);
	// run getEffectivenessMatrix with empty configuration to correctly initialize _first_swash_plate_servo_index
	ActuatorEffectiveness::Configuration configuration{};
	EffectivenessUpdateReason external_update = EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE;
	helicopter.getEffectivenessMatrix(configuration, external_update);

	Vector<float, 6> control_sp{};
	ActuatorEffectiveness::ActuatorVector actuator_sp{};
	ActuatorEffectiveness::ActuatorVector actuator_min{};
	actuator_min.setAll(0.f);
	ActuatorEffectiveness::ActuatorVector actuator_max{};
	actuator_max.setAll(1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 0.1f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), 0.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = 0.f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), 0.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.125f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .15f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.25f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .3f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.375f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .45f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.5f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .6f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.625f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .7f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.75f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .8f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -.875f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), .9f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), 1.f);

	control_sp(ActuatorEffectiveness::ControlAxis::THRUST_Z) = -1.1f;
	helicopter.updateSetpoint(control_sp, 0, actuator_sp, actuator_min, actuator_max);
	EXPECT_FLOAT_EQ(actuator_sp(0), 1.f);
}
