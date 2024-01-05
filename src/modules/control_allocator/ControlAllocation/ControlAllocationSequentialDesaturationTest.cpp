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

// Returns an effective matrix for a sample quad-copter configuration.
ActuatorEffectiveness::EffectivenessMatrix make_quad_effectiveness() {
	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	const auto geometry = make_quad_geometry();
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);
	return effectiveness;
}

// Configures a ControlAllocationSequentialDesaturation object for a sample quad-copter.
void setup_quad_allocator(ControlAllocationSequentialDesaturation &allocator) {
	const auto effectiveness = make_quad_effectiveness();
	matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> actuator_trim;
	matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> linearization_point;
	constexpr bool UPDATE_NORMALIZATION_SCALE{false};
	allocator.setEffectivenessMatrix(
		effectiveness,
		actuator_trim,
		linearization_point,
		ActuatorEffectiveness::NUM_ACTUATORS,
		UPDATE_NORMALIZATION_SCALE
	);
}

// Returns true if the 2 input values are within TOLERANCE of each other.
bool is_similar(float a, float b) {
	constexpr float TOLERANCE{1e-4};
	const auto diff = std::abs(a - b);
	return diff <= TOLERANCE;
}

} // namespace

// This tests that yaw-only control setpoint at zero actuator setpoint results in zero actuator
// allocation.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledOnlyYaw)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0;
	control_sp(ControlAllocation::ControlAxis::YAW) = 1;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = 0;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> zero;
	EXPECT_EQ(actuator_sp, zero);
}

// This tests that a control setpoint for z-thrust returns the desired actuator setpoint.
// Each motor should have an actuator setpoint that when summed together should be equal to
// control setpoint. 
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledThrustZ)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	constexpr float THRUST_Z_TOTAL{0.75};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0;
	control_sp(ControlAllocation::ControlAxis::YAW) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	constexpr float THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / 4};
	for (size_t i{0}; i < 4; ++i) {
		EXPECT_TRUE(is_similar(actuator_sp(i), THRUST_Z_PER_MOTOR));
	}
	for (size_t i{4}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_TRUE(is_similar(actuator_sp(i), 0));
	}
}

// This tests that a control setpoint for z-thrust returns the desired actuator setpoint.
// Each motor should have an actuator setpoint that when summed together should be equal to
// control setpoint. 
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledThrustAndYaw)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	constexpr float THRUST_Z_TOTAL{0.75};
	constexpr float YAW_CONTROL_SP{0.25};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0;
	control_sp(ControlAllocation::ControlAxis::YAW) = YAW_CONTROL_SP;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	constexpr float THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / 4};
	actuator_sp.print();
	for (size_t i{0}; i < 4; ++i) {
		EXPECT_TRUE(is_similar(actuator_sp(i), THRUST_Z_PER_MOTOR));
	}
	for (size_t i{4}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_TRUE(is_similar(actuator_sp(i), 0));
	}
}