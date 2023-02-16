/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
#include <ControlAllocationSequentialDesaturation.hpp>
#include <ActuatorEffectivenessRotors.hpp>

using namespace matrix;

TEST(ControlAllocationSequentialDesaturationTest, AllZeroCase)
{
	ControlAllocationSequentialDesaturation control_allocation;
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), (Vector<float, ControlAllocation::NUM_ACTUATORS>()));
	control_allocation.allocate();
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), (Vector<float, ControlAllocation::NUM_ACTUATORS>()));
}

TEST(ControlAllocationSequentialDesaturationTest, SetGetActuatorSetpoint)
{
	ControlAllocationSequentialDesaturation control_allocation;
	float actuator_setpoint_array[ControlAllocation::NUM_ACTUATORS] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
	Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_setpoint(actuator_setpoint_array);
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
	void SetUp() override
	{
		// Quadrotor x geometry
		ActuatorEffectivenessRotors::Geometry quadx_geometry{};
		quadx_geometry.num_rotors = 4;
		quadx_geometry.rotors[0].position = Vector3f(1.f, 1.f, 0.f); // clockwise motor numbering
		quadx_geometry.rotors[1].position = Vector3f(-1.f, 1.f, 0.f);
		quadx_geometry.rotors[2].position = Vector3f(-1.f, -1.f, 0.f);
		quadx_geometry.rotors[3].position = Vector3f(1.f, -1.f, 0.f);
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
		param_control_autosave(false); // Disable autosaving parameters to avoid busy loop in param_set()
		param_t param = param_find("MC_AIRMODE");
		param_set(param, &mode);
		_control_allocation.updateParams();
	}

	Vector4f allocate(float roll, float pitch, float yaw, float thrust)
	{
		Vector<float, ControlAllocation::NUM_AXES> control_setpoint;
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
		// All unused actuators shall stay zero
		static constexpr uint8_t NUM_UNUSED_ACTUATORS = ControlAllocation::NUM_ACTUATORS - 4;
		EXPECT_EQ((Vector<float, NUM_UNUSED_ACTUATORS>(_control_allocation.getActuatorSetpoint().slice<NUM_UNUSED_ACTUATORS, 1>
				(4, 0))), (Vector<float, NUM_UNUSED_ACTUATORS>()));
		return Vector4f(_control_allocation.getActuatorSetpoint().slice<4, 1>(0, 0));
	}

	void collectiveThrustTest(const float thrust)
	{
		EXPECT_EQ(allocate(0.f, 0.f, 0.f, -thrust), Vector4f(thrust, thrust, thrust, thrust));
	}

	TestControlAllocationSequentialDesaturation _control_allocation;
};

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, Zero)
{
	EXPECT_EQ(allocate(0.f, 0.f, 0.f, 0.f), Vector4f());
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, CollectiveThrust)
{
	for (float thrust = 0.f; thrust >= 1.f; thrust += .1f) {
		collectiveThrustTest(thrust);
	}
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYaw)
{
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, -.5f), Vector4f(.25f, .25f, .75f, .75f));
	EXPECT_EQ(allocate(-1.f, 0.f, 0.f, -.5f), Vector4f(.75f, .75f, .25f, .25f));
	EXPECT_EQ(allocate(0.f, 1.f, 0.f, -.5f), Vector4f(.75f, .25f, .25f, .75f));
	EXPECT_EQ(allocate(0.f, -1.f, 0.f, -.5f), Vector4f(.25f, .75f, .75f, .25f));
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, -.5f), Vector4f(.75f, .25f, .75f, .25f));
	EXPECT_EQ(allocate(0.f, 0.f, -1.f, -.5f), Vector4f(.25f, .75f, .25f, .75f));
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, RollPitchYawFullThrust)
{
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

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, TEMP)
{
	setAirmode(2); // Roll, pitch and yaw airmode
	EXPECT_EQ(allocate(1.f, 0.f, 0.f, 0.f), Vector4f(0.f, 0.f, 0.5f, 0.5f));
}
