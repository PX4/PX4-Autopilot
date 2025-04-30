/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
#include <ActuatorEffectivenessRotors.hpp>
#include "ControlAllocationSequentialDesaturation.hpp"

using namespace matrix;
using ActuatorVector = ControlAllocation::ActuatorVector;

TEST(ControlAllocationSequentialDesaturationTest, AllZeroCase)
{
	ControlAllocationSequentialDesaturation control_allocation;
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), ActuatorVector());
	control_allocation.allocate();
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), ActuatorVector());
}

TEST(ControlAllocationSequentialDesaturationTest, SetGetActuatorSetpoint)
{
	ControlAllocationSequentialDesaturation control_allocation;
	float actuator_setpoint_array[ControlAllocation::NUM_ACTUATORS] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
	ActuatorVector actuator_setpoint(actuator_setpoint_array);
	control_allocation.setActuatorSetpoint(actuator_setpoint);
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), actuator_setpoint);
}

// Make protected updateParams() function available to the unit test to change airmode after initialization
class TestControlAllocationSequentialDesaturation : public ::ControlAllocationSequentialDesaturation
{
public:
	void updateParams() { ControlAllocationSequentialDesaturation::updateParams(); }
};

class ControlAllocationSequentialDesaturationTestQuadX : public ::testing::Test
{
public:
	TestControlAllocationSequentialDesaturation _control_allocation;

	void SetUp() override
	{
		param_control_autosave(false); // Disable autosaving parameters to avoid busy loop in param_set()

		// Quadrotor x geometry
		ActuatorEffectivenessRotors::Geometry quadx_geometry{};
		quadx_geometry.num_rotors = 4;
		quadx_geometry.rotors[0].position = {1.f, 1.f, 0.f}; // clockwise motor numbering
		quadx_geometry.rotors[1].position = {-1.f, 1.f, 0.f};
		quadx_geometry.rotors[2].position = {-1.f, -1.f, 0.f};
		quadx_geometry.rotors[3].position = {1.f, -1.f, 0.f};
		quadx_geometry.rotors[0].moment_ratio = 1.f;
		quadx_geometry.rotors[1].moment_ratio = -1.f;
		quadx_geometry.rotors[2].moment_ratio = 1.f;
		quadx_geometry.rotors[3].moment_ratio = -1.f;

		for (int i = 0; i < 4; ++i) {
			quadx_geometry.rotors[i].axis = Vector3f(0.f, 0.f, -1.f); // thrust downwards
			quadx_geometry.rotors[i].thrust_coef = 1.f;
			quadx_geometry.rotors[i].tilt_index = -1;
		}

		// Compute actuator effectiveness
		ActuatorEffectiveness::Configuration actuator_configuration{};
		int num_actuators = ActuatorEffectivenessRotors::computeEffectivenessMatrix(quadx_geometry,
				    actuator_configuration.effectiveness_matrices[0],
				    actuator_configuration.num_actuators_matrix[0]);
		actuator_configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);

		// Load effectiveness into allocation
		_control_allocation.setEffectivenessMatrix(actuator_configuration.effectiveness_matrices[0],
				actuator_configuration.trim[0], actuator_configuration.linearization_point[0],
				actuator_configuration.num_actuators_matrix[0], true /*update_normalization_scale*/);
	}

	void setAirmode(const int32_t mode)
	{
		param_t param = param_find("MC_AIRMODE");
		param_set(param, &mode);
		_control_allocation.updateParams();
	}

	Vector4f allocate(float roll, float pitch, float yaw, float thrust)
	{
		Vector<float, ControlAllocation::NUM_AXES> control_setpoint{};
		control_setpoint(ControlAllocation::ControlAxis::ROLL) = roll;
		control_setpoint(ControlAllocation::ControlAxis::PITCH) = pitch;
		control_setpoint(ControlAllocation::ControlAxis::YAW) = yaw;
		control_setpoint(ControlAllocation::ControlAxis::THRUST_Z) = thrust;
		_control_allocation.setControlSetpoint(control_setpoint);
		_control_allocation.allocate();
		return getQuadOutputs();
	}

	Vector4f getQuadOutputs()
	{
		const ActuatorVector &actuator_setpoint = _control_allocation.getActuatorSetpoint();
		// All unused actuators shall stay zero
		static constexpr uint8_t NUM_UNUSED_ACTUATORS = ControlAllocation::NUM_ACTUATORS - 4;
		EXPECT_EQ(
			(Vector<float, NUM_UNUSED_ACTUATORS>(actuator_setpoint.slice<NUM_UNUSED_ACTUATORS, 1>(4, 0))),
			(Vector<float, NUM_UNUSED_ACTUATORS>())
		);
		return Vector4f(actuator_setpoint.slice<4, 1>(0, 0));
	}
};

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, Zero)
{
	EXPECT_EQ(allocate(0.f, 0.f, 0.f, 0.f), Vector4f());
}


TEST_F(ControlAllocationSequentialDesaturationTestQuadX, CollectiveThrust)
{
	for (float thrust = 0.f; thrust <= (1.f + FLT_EPSILON); thrust += .1f) {
		EXPECT_EQ(allocate(0.f, 0.f, 0.f, -thrust), Vector4f(thrust, thrust, thrust, thrust));
	}
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYaw)
{
	setAirmode(0); // Airmode disabled
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, -.5f), Vector4f(.25f, .25f, .75f, .75f));
	EXPECT_EQ(allocate(-1.f, 0.f, 0.f, -.5f), Vector4f(.75f, .75f, .25f, .25f));
	EXPECT_EQ(allocate(0.f, 1.f, 0.f, -.5f), Vector4f(.75f, .25f, .25f, .75f));
	EXPECT_EQ(allocate(0.f, -1.f, 0.f, -.5f), Vector4f(.25f, .75f, .75f, .25f));
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, -.5f), Vector4f(.75f, .25f, .75f, .25f));
	EXPECT_EQ(allocate(0.f, 0.f, -1.f, -.5f), Vector4f(.25f, .75f, .25f, .75f));
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYawFullThrust)
{
	setAirmode(0); // Airmode disabled
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, -1.f), Vector4f(.5f, .5f, 1.f, 1.f));
	EXPECT_EQ(allocate(-1.f, 0.f, 0.f, -1.f), Vector4f(1.f, 1.f, .5f, .5f));
	EXPECT_EQ(allocate(0.f, 1.f, 0.f, -1.f), Vector4f(1.f, .5f, .5f, 1.f));
	EXPECT_EQ(allocate(0.f, -1.f, 0.f, -1.f), Vector4f(.5f, 1.f, 1.f, .5f));
	// There is a special case to deprioritize yaw down to 30% authority with maximum thrust
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, -1.f), Vector4f(1.f, .7f, 1.f, .7f));
	EXPECT_EQ(allocate(0.f, 0.f, -1.f, -1.f), Vector4f(.7f, 1.f, .7f, 1.f));
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYawZeroThrust)
{
	setAirmode(0); // Airmode disabled
	// No axis is allocated
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, 0.f), Vector4f());
	EXPECT_EQ(allocate(-1.f, 0.f, 0.f, 0.f), Vector4f());
	EXPECT_EQ(allocate(0.f, 1.f, 0.f, 0.f), Vector4f());
	EXPECT_EQ(allocate(0.f, -1.f, 0.f, 0.f), Vector4f());
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, 0.f), Vector4f());
	EXPECT_EQ(allocate(0.f, 0.f, -1.f, 0.f), Vector4f());
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYawZeroThrustAirmodeRP)
{
	setAirmode(1); // Roll and pitch airmode
	// Roll and pitch get fully allocated
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, 0.f), Vector4f(0.f, 0.f, 0.5f, 0.5f));
	EXPECT_EQ(allocate(-1.f, 0.f, 0.f, 0.f), Vector4f(0.5f, 0.5f, 0.f, 0.f));
	EXPECT_EQ(allocate(0.f, 1.f, 0.f, 0.f), Vector4f(0.5f, 0.f, 0.f, 0.5f));
	EXPECT_EQ(allocate(0.f, -1.f, 0.f, 0.f), Vector4f(0.f, 0.5f, 0.5f, 0.f));
	// Yaw is not allocated
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, 0.f), Vector4f());
	EXPECT_EQ(allocate(0.f, 0.f, -1.f, 0.f), Vector4f());
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYawZeroThrustAirmodeRPY)
{
	setAirmode(2); // Roll, pitch and yaw airmode
	// All axis are fully allocated
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, 0.f), Vector4f(0.f, 0.f, 0.5f, 0.5f));
	EXPECT_EQ(allocate(-1.f, 0.f, 0.f, 0.f), Vector4f(0.5f, 0.5f, 0.f, 0.f));
	EXPECT_EQ(allocate(0.f, 1.f, 0.f, 0.f), Vector4f(0.5f, 0.f, 0.f, 0.5f));
	EXPECT_EQ(allocate(0.f, -1.f, 0.f, 0.f), Vector4f(0.f, 0.5f, 0.5f, 0.f));
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, 0.f), Vector4f(.5f, 0.f, .5f, 0.f));
	EXPECT_EQ(allocate(0.f, 0.f, -1.f, 0.f), Vector4f(0.f, .5f, 0.f, .5f));
}

namespace
{

// Makes and returns a Geometry object for a "standard" quad-x quadcopter.
ActuatorEffectivenessRotors::Geometry make_quad_x_geometry()
{
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
ActuatorEffectiveness::EffectivenessMatrix make_quad_x_effectiveness()
{
	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	const auto geometry = make_quad_x_geometry();
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);
	return effectiveness;
}

// Configures a ControlAllocationSequentialDesaturation object for a sample quad-copter.
void setup_quad_allocator(ControlAllocationSequentialDesaturation &allocator)
{
	const auto effectiveness = make_quad_x_effectiveness();
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

static constexpr float EXPECT_NEAR_TOL{1e-4f};

} // namespace

// This tests that yaw-only control setpoint at zero actuator setpoint results in zero actuator
// allocation.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledOnlyYaw)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0.f;
	control_sp(ControlAllocation::ControlAxis::YAW) = 1.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = 0.f;
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
	// Negative, because +z is "downward".
	constexpr float THRUST_Z_TOTAL{-0.75f};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0.f;
	control_sp(ControlAllocation::ControlAxis::YAW) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	constexpr int MOTOR_COUNT{4};
	constexpr float THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT};

	for (int i{0}; i < MOTOR_COUNT; ++i) {
		EXPECT_NEAR(actuator_sp(i), THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	}

	for (int i{MOTOR_COUNT}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), 0.f, EXPECT_NEAR_TOL);
	}
}

// This tests that a control setpoint for z-thrust + yaw returns the desired actuator setpoint.
// This test does not saturate the yaw response.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledThrustAndYaw)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	// Negative, because +z is "downward".
	constexpr float THRUST_Z_TOTAL{-0.75f};
	// This is low enough to not saturate the motors.
	constexpr float YAW_CONTROL_SP{0.02f};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0.f;
	control_sp(ControlAllocation::ControlAxis::YAW) = YAW_CONTROL_SP;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	// This value is based off of the effectiveness matrix. If the effectiveness matrix is changed,
	// this will need to be changed.
	constexpr float YAW_EFFECTIVENESS_FACTOR{5.f};
	constexpr float YAW_DIFF_PER_MOTOR{YAW_CONTROL_SP * YAW_EFFECTIVENESS_FACTOR};
	// At yaw condition, there will be 2 different actuator values.
	constexpr int MOTOR_COUNT{4};
	constexpr float HIGH_THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT + YAW_DIFF_PER_MOTOR};
	constexpr float LOW_THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT - YAW_DIFF_PER_MOTOR};

	for (int i{0}; i < MOTOR_COUNT / 2; ++i) {
		EXPECT_NEAR(actuator_sp(i), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	}

	for (int i{MOTOR_COUNT / 2}; i < MOTOR_COUNT; ++i) {
		EXPECT_NEAR(actuator_sp(i), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	}

	for (int i{MOTOR_COUNT}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), 0.f, EXPECT_NEAR_TOL);
	}
}

// This tests that a control setpoint for z-thrust + yaw returns the desired actuator setpoint.
// This test saturates the yaw response, but does not reduce total thrust.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledThrustAndSaturatedYaw)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	// Negative, because +z is "downward".
	constexpr float THRUST_Z_TOTAL{-0.75f};
	// This is arbitrarily high to trigger strongest possible (saturated) yaw response.
	constexpr float YAW_CONTROL_SP{0.25f};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0.f;
	control_sp(ControlAllocation::ControlAxis::YAW) = YAW_CONTROL_SP;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	// At max yaw, only 2 motors will carry all of the thrust.
	constexpr int YAW_MOTORS{2};
	constexpr float THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / YAW_MOTORS};

	for (int i{0}; i < YAW_MOTORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	}

	for (int i{YAW_MOTORS}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), 0.f, EXPECT_NEAR_TOL);
	}
}

// This tests that a control setpoint for z-thrust + pitch returns the desired actuator setpoint.
// This test does not saturate the pitch response.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledThrustAndPitch)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	// Negative, because +z is "downward".
	constexpr float THRUST_Z_TOTAL{-0.75f};
	// This is low enough to not saturate the motors.
	constexpr float PITCH_CONTROL_SP{0.1f};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = PITCH_CONTROL_SP;
	control_sp(ControlAllocation::ControlAxis::YAW) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	// This value is based off of the effectiveness matrix. If the effectiveness matrix is changed,
	// this will need to be changed.
	constexpr int MOTOR_COUNT{4};
	constexpr float PITCH_DIFF_PER_MOTOR{PITCH_CONTROL_SP / MOTOR_COUNT};
	// At control set point, there will be 2 different actuator values.
	constexpr float HIGH_THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT + PITCH_DIFF_PER_MOTOR};
	constexpr float LOW_THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT - PITCH_DIFF_PER_MOTOR};
	EXPECT_NEAR(actuator_sp(0), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(1), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(2), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(3), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);

	for (int i{MOTOR_COUNT}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), 0.f, EXPECT_NEAR_TOL);
	}
}

// This tests that a control setpoint for z-thrust + yaw returns the desired actuator setpoint.
// This test saturates yaw and demonstrates reduction of thrust for yaw.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledReducedThrustAndYaw)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	// Negative, because +z is "downward".
	constexpr float DESIRED_THRUST_Z_PER_MOTOR{0.8f};
	constexpr int MOTOR_COUNT{4};
	constexpr float THRUST_Z_TOTAL{-DESIRED_THRUST_Z_PER_MOTOR * MOTOR_COUNT};
	// This is arbitrarily high to trigger strongest possible (saturated) yaw response.
	constexpr float YAW_CONTROL_SP{1.f};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0.f;
	control_sp(ControlAllocation::ControlAxis::YAW) = YAW_CONTROL_SP;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	// In the case of yaw saturation, thrust per motor will be reduced by the hard-coded
	// magic-number yaw margin of 0.15f.
	constexpr float YAW_MARGIN{0.15f}; // get this from a centralized source when available.
	constexpr float YAW_DIFF_PER_MOTOR{1.0f + YAW_MARGIN - DESIRED_THRUST_Z_PER_MOTOR};
	// At control set point, there will be 2 different actuator values.
	constexpr float HIGH_THRUST_Z_PER_MOTOR{DESIRED_THRUST_Z_PER_MOTOR + YAW_DIFF_PER_MOTOR - YAW_MARGIN};
	constexpr float LOW_THRUST_Z_PER_MOTOR{DESIRED_THRUST_Z_PER_MOTOR - YAW_DIFF_PER_MOTOR - YAW_MARGIN};
	EXPECT_NEAR(actuator_sp(0), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(1), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(2), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(3), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);

	for (int i{MOTOR_COUNT}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), 0.f, EXPECT_NEAR_TOL);
	}
}

// This tests that a control setpoint for z-thrust + pitch returns the desired actuator setpoint.
// This test saturates the pitch response such that thrust is reduced to (partially) compensate.
TEST(ControlAllocationSequentialDesaturationTest, AirmodeDisabledReducedThrustAndPitch)
{
	ControlAllocationSequentialDesaturation allocator;
	setup_quad_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;
	// Negative, because +z is "downward".
	constexpr float THRUST_Z_TOTAL{-0.75f * 4.f};
	// This is high enough to saturate the pitch control.
	constexpr float PITCH_CONTROL_SP{2.f};
	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = PITCH_CONTROL_SP;
	control_sp(ControlAllocation::ControlAxis::YAW) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = THRUST_Z_TOTAL;
	allocator.setControlSetpoint(control_sp);

	// Since MC_AIRMODE was not set explicitly, assume airmode is disabled.
	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();
	constexpr int MOTOR_COUNT{4};
	// The maximum actuator value is
	// 	THRUST_Z_TOTAL / MOTOR_COUNT + PITCH_CONTROL_SP / MOTOR_COUNT.
	// The amount over 1 is the amount that each motor is reduced by.
	// At control set point, there will be 2 different actuator values.
	constexpr float OVERAGE_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT + PITCH_CONTROL_SP / MOTOR_COUNT - 1};
	EXPECT_TRUE(OVERAGE_PER_MOTOR > 0.f);
	constexpr float HIGH_THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT + PITCH_CONTROL_SP / MOTOR_COUNT - OVERAGE_PER_MOTOR};
	constexpr float LOW_THRUST_Z_PER_MOTOR{-THRUST_Z_TOTAL / MOTOR_COUNT - PITCH_CONTROL_SP / MOTOR_COUNT - OVERAGE_PER_MOTOR};
	EXPECT_NEAR(actuator_sp(0), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(1), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(2), HIGH_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);
	EXPECT_NEAR(actuator_sp(3), LOW_THRUST_Z_PER_MOTOR, EXPECT_NEAR_TOL);

	for (int i{MOTOR_COUNT}; i < ActuatorEffectiveness::NUM_ACTUATORS; ++i) {
		EXPECT_NEAR(actuator_sp(i), 0.f, EXPECT_NEAR_TOL);
	}
}
