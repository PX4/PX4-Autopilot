/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationSequentialDesaturationTest.cpp
 *
 * Tests for Control Allocation Sequential Desaturation Algorithms
 *
 */

#include <gtest/gtest.h>
#include <ControlAllocationSequentialDesaturation.hpp>
#include <../ActuatorEffectiveness/ActuatorEffectivenessRotors.hpp>

using namespace matrix;

namespace {

// Makes and returns a Geometry object for a "standard" quadcopter.
ActuatorEffectivenessRotors::Geometry make_quad_geometry() {
	ActuatorEffectivenessRotors::Geometry geometry = {};
	geometry.rotors[0].position(0) = 1.0f;
	geometry.rotors[0].position(1) = 1.0f;
	geometry.rotors[0].position(2) = 0.0f;
	geometry.rotors[0].axis(0) = 0.0f;
	geometry.rotors[0].axis(1) = 0.0f;
	geometry.rotors[0].axis(2) = -1.0f;
	geometry.rotors[0].thrust_coef = 1.0f;
	geometry.rotors[0].moment_ratio = 0.05f;

	geometry.rotors[1].position(0) = -1.0f;
	geometry.rotors[1].position(1) = -1.0f;
	geometry.rotors[1].position(2) = 0.0f;
	geometry.rotors[1].axis(0) = 0.0f;
	geometry.rotors[1].axis(1) = 0.0f;
	geometry.rotors[1].axis(2) = -1.0f;
	geometry.rotors[1].thrust_coef = 1.0f;
	geometry.rotors[1].moment_ratio = 0.05f;

	geometry.rotors[2].position(0) = 1.0f;
	geometry.rotors[2].position(1) = -1.0f;
	geometry.rotors[2].position(2) = 0.0f;
	geometry.rotors[2].axis(0) = 0.0f;
	geometry.rotors[2].axis(1) = 0.0f;
	geometry.rotors[2].axis(2) = -1.0f;
	geometry.rotors[2].thrust_coef = 1.0f;
	geometry.rotors[2].moment_ratio = -0.05f;

	geometry.rotors[3].position(0) = -1.0f;
	geometry.rotors[3].position(1) = 1.0f;
	geometry.rotors[3].position(2) = 0.0f;
	geometry.rotors[3].axis(0) = 0.0f;
	geometry.rotors[3].axis(1) = 0.0f;
	geometry.rotors[3].axis(2) = -1.0f;
	geometry.rotors[3].thrust_coef = 1.0f;
	geometry.rotors[3].moment_ratio = -0.05f;

	geometry.num_rotors = 4;

	return geometry;
}

ActuatorEffectiveness::EffectivenessMatrix make_quad_effectiveness() {
	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	const auto geometry = make_quad_geometry();
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);
	return effectiveness;
}

} // namespace

// This tests that yaw-only control setpoint at zero actuator setpoint results in zero actuator
// allocation.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledOnlyYaw)
{
	ControlAllocationSequentialDesaturation method;

	const auto effectiveness = make_quad_effectiveness();
	matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> actuator_trim;
	matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> linearization_point;
	constexpr bool UPDATE_NORMALIZATION_SCALE{false};
	method.setEffectivenessMatrix(
		effectiveness,
		actuator_trim,
		linearization_point,
		ActuatorEffectiveness::NUM_ACTUATORS,
		UPDATE_NORMALIZATION_SCALE
	);

	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	control_sp(ControlAxis::ROLL) = 0;
	control_sp(ControlAxis::PITCH) = 0;
	control_sp(ControlAxis::YAW) = 1;
	control_sp(ControlAxis::THRUST_X) = 0;
	control_sp(ControlAxis::THRUST_Y) = 0;
	control_sp(ControlAxis::THRUST_Z) = 0;
	method.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	method.allocate();

	const auto &actuator_sp = method.getActuatorSetpoint()
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> zero;
	EXPECT_EQ(actuator_sp, zero);
 /*
	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
 */
}
