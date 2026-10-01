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

class ControlAllocationSequentialDesaturationTestQuadX : public ::testing::Test
{
public:
	static constexpr uint8_t NUM_ACTUATORS = 4;
	ControlAllocationSequentialDesaturation _control_allocation;

	void SetUp() override
	{
		param_control_autosave(false); // Disable autosaving parameters to avoid busy loop in param_set()
		setAirmode(0); // No airmode by default

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
		EXPECT_EQ(num_actuators, NUM_ACTUATORS);
		actuator_configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);

		// Load effectiveness into allocation
		_control_allocation.setEffectivenessMatrix(actuator_configuration.effectiveness_matrices[0],
				actuator_configuration.trim[0], actuator_configuration.linearization_point[0],
				actuator_configuration.num_actuators_matrix[0], true /*update_normalization_scale*/);
	}

	void setAirmode(const int32_t mode)
	{
		float lim = 0.f;
		bool yaw = false;

		switch (mode) {
		case 1: lim = 1.f; yaw = false; break;

		case 2: lim = 1.f; yaw = true; break;

		default: lim = 0.f; yaw = false; break;
		}

		setAirmodeParams(lim, yaw);
	}

	void setAirmodeParams(const float lim, const bool yaw)
	{
		const int32_t yaw_param = yaw ? 1 : 0;
		param_set(param_find("MC_AIRMODE_LIM"), &lim);
		param_set(param_find("MC_AIRMODE_YAW"), &yaw_param);
		_control_allocation.updateParameters();
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

// Make constant available, see https://stackoverflow.com/questions/42756443/undefined-reference-with-gtest
constexpr uint8_t ControlAllocationSequentialDesaturationTestQuadX::NUM_ACTUATORS;

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

// This tests that yaw-only control setpoint at zero actuator setpoint results in zero actuator
// allocation.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledOnlyYaw)
{
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, 0.f), Vector4f(0.f, 0.f, 0.f, 0.f));
}

// This tests that a control setpoint for z-thrust returns the desired actuator setpoint.
// Each motor should have an actuator setpoint that when summed together should be equal to
// control setpoint.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledThrustZ)
{
	constexpr float THRUST = 0.75f;
	EXPECT_EQ(allocate(0.f, 0.f, 0.f, -THRUST), Vector4f(THRUST, THRUST, THRUST, THRUST));
}

// This tests that a control setpoint for z-thrust + yaw returns the desired actuator setpoint.
// This test does not saturate the yaw response.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledThrustAndYaw)
{
	constexpr float THRUST = 0.75f;
	constexpr float YAW_TORQUE = 0.02f;
	constexpr float YAW = YAW_TORQUE / NUM_ACTUATORS;
	EXPECT_EQ(allocate(0.f, 0.f, YAW_TORQUE, -THRUST), Vector4f(THRUST + YAW, THRUST - YAW, THRUST + YAW, THRUST - YAW));
}

// This tests that a control setpoint for z-thrust + yaw returns the desired actuator setpoint.
// This test saturates the yaw response, but does not reduce total thrust.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledThrustAndSaturatedYaw)
{
	constexpr float THRUST = 0.75f;
	constexpr float YAW_TORQUE = 1.f;
	constexpr float YAW = YAW_TORQUE / NUM_ACTUATORS;
	EXPECT_EQ(allocate(0.f, 0.f, YAW_TORQUE, -THRUST), Vector4f(THRUST + YAW, THRUST - YAW, THRUST + YAW, THRUST - YAW));
}

// This tests that a control setpoint for z-thrust + pitch returns the desired actuator setpoint.
// This test does not saturate the pitch response.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledThrustAndPitch)
{
	constexpr float THRUST = 0.75f;
	constexpr float PITCH_TORQUE = 0.1f;
	constexpr float PITCH = PITCH_TORQUE / NUM_ACTUATORS;
	EXPECT_EQ(allocate(0.f, PITCH_TORQUE, 0.f, -THRUST),
		  Vector4f(THRUST + PITCH, THRUST - PITCH, THRUST - PITCH, THRUST + PITCH));
}

// This tests that a control setpoint for z-thrust + yaw returns the desired actuator setpoint.
// This test saturates yaw and demonstrates reduction of thrust for yaw.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledReducedThrustAndYaw)
{
	constexpr float YAW_MARGIN = ControlAllocationSequentialDesaturation::MINIMUM_YAW_MARGIN;
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, -3.2f), Vector4f(1.f, 1.f - (2.f * YAW_MARGIN), 1.f, 1.f - (2.f * YAW_MARGIN)));
}

// This tests that a control setpoint for z-thrust + pitch returns the desired actuator setpoint.
// This test saturates the pitch response such that thrust is reduced to (partially) compensate.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeDisabledReducedThrustAndPitch)
{
	EXPECT_EQ(allocate(0.f, 2.f, 0.f, -3.f), Vector4f(1.f, 0.f, 0.f, 1.f));
}

// MC_AIRMODE_LIM = 0 (set directly, not via the MC_AIRMODE enum wrapper) must reproduce the
// airmode-disabled outputs bit-exactly. Cross-checks the float-only API path.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeLimZeroMatchesDisabled)
{
	setAirmodeParams(0.f, false);
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.f, 0.f, 0.2f, 0.2f));   // row 37
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.100f), Vector4f(0.f, 0.2f, 0.2f, 0.f));  // row 42
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -1.000f), Vector4f(1.f, 0.7f, 1.f, 0.7f));   // row 50
}

// MC_AIRMODE_LIM = 1 (set directly) must reproduce roll/pitch airmode.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeLimOneMatchesRP)
{
	setAirmodeParams(1.f, false);
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.f, 0.f, 0.5f, 0.5f));   // row 37
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.100f), Vector4f(0.f, 0.5f, 0.5f, 0.f));  // row 42
}

// MC_AIRMODE_LIM = 1, MC_AIRMODE_YAW = 1 must reproduce roll/pitch/yaw airmode.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeLimOneAndYawMatchRPY)
{
	setAirmodeParams(1.f, true);
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.000f), Vector4f(0.f, 0.f, 0.f, 1.f));    // row 51 RPY
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.000f), Vector4f(0.5f, 0.f, 0.5f, 0.f));   // row 46 RPY
}

// Sweep MC_AIRMODE_LIM ∈ {0, 0.05, 0.10, 0.15, 1.0} for the saturating input
// (roll=1, thrust=-0.1) and verify each motor's output is non-decreasing in the
// limit. Endpoints match the airmode-disabled and roll/pitch-airmode row 37 values.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeLimMonotonicity)
{
	const float sweep[] = {0.f, 0.05f, 0.10f, 0.15f, 1.f};
	Vector4f prev{};

	for (size_t i = 0; i < sizeof(sweep) / sizeof(sweep[0]); ++i) {
		setAirmodeParams(sweep[i], false);
		Vector4f out = allocate(1.f, 0.f, 0.f, -0.1f);

		if (i == 0) {
			EXPECT_EQ(out, Vector4f(0.f, 0.f, 0.2f, 0.2f)); // airmode disabled
		}

		if (i + 1 == sizeof(sweep) / sizeof(sweep[0])) {
			EXPECT_EQ(out, Vector4f(0.f, 0.f, 0.5f, 0.5f)); // roll/pitch airmode
		}

		if (i > 0) {
			for (int m = 0; m < 4; ++m) {
				EXPECT_GE(out(m), prev(m)) << "non-monotonic at MC_AIRMODE_LIM=" << sweep[i] << " motor=" << m;
			}
		}

		prev = out;
	}
}

// The yaw path is structurally different with yaw airmode off (deferred path with the hidden
// 15% MINIMUM_YAW_MARGIN) vs on (yaw mixed into the initial accumulation). Pin both sides to
// detect anyone "smoothing" them together in a way that breaks either endpoint.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, YawAirmodeSwitchesYawPath)
{
	setAirmodeParams(1.f, false);
	const Vector4f deferred_path = allocate(0.f, 0.f, 1.f, -1.f);
	EXPECT_EQ(deferred_path, Vector4f(1.f, 0.7f, 1.f, 0.7f)); // 30% yaw via 15% margin trick

	setAirmodeParams(1.f, true);
	const Vector4f sum_path = allocate(0.f, 0.f, 1.f, -1.f);
	EXPECT_EQ(sum_path, Vector4f(1.f, 0.5f, 1.f, 0.5f));      // 50% yaw via priority sum

	EXPECT_FALSE(deferred_path == sum_path) << "yaw airmode off and on must differ";
}

// Roll/pitch airmode (MC_AIRMODE_YAW=0) folds yaw into the thrust desaturation when yaw relieves the
// saturating actuator, removing the collective over-command of pure sequential allocation (the
// "#16" case). Roll, pitch and yaw are still delivered exactly; only the unnecessary extra
// collective is dropped, so the result matches full roll/pitch/yaw airmode for this command.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, DeferredYawFoldedWhenItReducesThrust)
{
	setAirmodeParams(1.f, false);
	const Vector4f folded = allocate(0.05f, 0.05f, -0.025f, 0.f);
	EXPECT_EQ(folded, Vector4f(0.0125f, 0.f, 0.0125f, 0.05f));

	// Strictly less total collective than the old deferred-yaw allocation (0.01875,...,0.05625).
	EXPECT_LT(folded(0) + folded(1) + folded(2) + folded(3), 0.1f);

	// Equals what full roll/pitch/yaw airmode (yaw already in the sum) produces for this command.
	setAirmodeParams(1.f, true);
	EXPECT_EQ(allocate(0.05f, 0.05f, -0.025f, 0.f), folded);
}

// When yaw would worsen the saturating actuator it stays deferred (deprioritized yaw).
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, DeferredYawKeptWhenYawWorsens)
{
	setAirmodeParams(1.f, false);
	EXPECT_EQ(allocate(0.05f, 0.05f, 0.025f, 0.f), Vector4f(0.025f, 0.f, 0.025f, 0.05f));
}

// With pitch and yaw saturating both bounds, the thrust-shift gain used to decide the fold was an
// exact tie over this whole throttle range, so float rounding switched between the folded and the
// deferred allocation from one throttle step to the next (at LIM 0.2, pitch 0.5 <-> 0.2 per motor).
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, DeferredYawFoldStableAcrossTie)
{
	for (int i = 0; i <= 20; ++i) {
		const float thrust = 0.4f + 0.01f * i;

		setAirmodeParams(1.f, false);
		EXPECT_EQ(allocate(0.f, 4.f, 1.2f, -thrust), Vector4f(1.5f, -0.5f, -0.5f, 1.5f)) << "thrust " << thrust;

		setAirmodeParams(0.2f, false);
		EXPECT_EQ(allocate(0.f, 4.f, 1.2f, -thrust), Vector4f(1.f, 0.f, 0.f, 1.f)) << "thrust " << thrust;
	}
}

// MC_AIRMODE_LIM = 0 is airmode disabled whatever MC_AIRMODE_YAW says: yaw airmode spends the
// limit's thrust budget, so without one it must not move yaw into the initial sum either (which
// would, for example, give up collective for yaw at full throttle).
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, YawAirmodeWithoutLimitMatchesDisabled)
{
	setAirmodeParams(0.f, true);
	EXPECT_EQ(allocate(0.f, 0.f, 1.f, -1.f), Vector4f(1.f, 0.7f, 1.f, 0.7f)); // row 50, disabled

	const float torques[] = {-4.f, -1.f, -0.2f, 0.f, 0.2f, 1.f, 4.f};
	const float thrusts[] = {0.f, 0.1f, 0.5f, 0.9f, 1.f};

	for (const float roll : torques) {
		for (const float pitch : torques) {
			for (const float yaw : torques) {
				for (const float thrust : thrusts) {
					setAirmodeParams(0.f, false);
					const Vector4f disabled = allocate(roll, pitch, yaw, -thrust);
					setAirmodeParams(0.f, true);
					EXPECT_EQ(allocate(roll, pitch, yaw, -thrust), disabled)
							<< "roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << " thrust=" << thrust;
				}
			}
		}
	}
}

// With yaw airmode, yaw spends the same MC_AIRMODE_LIM budget as roll/pitch, so once the outputs
// are clipped the way ControlAllocator does the mean thrust never exceeds the command plus the
// limit, for either yaw sign. A separate fractional yaw limit used to leave yaw saturated instead,
// and the clipping then added collective: 0.49 mean thrust instead of 0.3 in the first case below.
// Checked for yaw with one of roll/pitch: with all three axes saturating, the single-axis passes
// can leave outputs they cannot move (their per-output gains cancel), whatever MC_AIRMODE_YAW is.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, YawAirmodeThrustBoundedByLimit)
{
	setAirmodeParams(0.2f, true);
	EXPECT_EQ(allocate(0.f, 0.f, 2.8f, -0.1f), Vector4f(0.6f, 0.f, 0.6f, 0.f));
	EXPECT_EQ(allocate(0.f, 0.f, -2.8f, -0.1f), Vector4f(0.f, 0.6f, 0.f, 0.6f));

	const float torques[] = {-4.f, -2.f, -1.f, -0.4f, 0.f, 0.4f, 1.f, 2.f, 4.f};
	const float thrusts[] = {0.f, 0.05f, 0.1f, 0.3f, 0.5f};

	auto clipped_mean = [](const Vector4f & out) {
		float mean = 0.f;

		for (int i = 0; i < 4; ++i) { mean += 0.25f * fmaxf(0.f, fminf(1.f, out(i))); }

		return mean;
	};

	for (const float lim : {0.1f, 0.2f, 0.5f}) {
		for (const bool yaw_airmode : {false, true}) {
			setAirmodeParams(lim, yaw_airmode);

			for (const float torque : torques) {
				for (const float yaw : torques) {
					for (const float thrust : thrusts) {
						EXPECT_LE(clipped_mean(allocate(torque, 0.f, yaw, -thrust)), thrust + lim + 1e-4f)
								<< "LIM=" << lim << " yaw airmode=" << yaw_airmode << " roll=" << torque << " yaw=" << yaw
								<< " thrust=" << thrust;
						EXPECT_LE(clipped_mean(allocate(0.f, torque, yaw, -thrust)), thrust + lim + 1e-4f)
								<< "LIM=" << lim << " yaw airmode=" << yaw_airmode << " pitch=" << torque << " yaw=" << yaw
								<< " thrust=" << thrust;
					}
				}
			}
		}
	}
}

// Yaw is the least important axis: with yaw airmode and a limited budget, remaining saturation is
// taken from yaw first and from roll/pitch only once yaw is gone. Desaturating roll first used to
// drop all of the roll (0, 0, 0.6, 0.6 -> 0.6, 0, 0.6, 0) to keep yaw, and could even reverse it.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, YawAirmodeGivesUpYawBeforeRollPitch)
{
	setAirmodeParams(0.2f, true);

	// Roll and yaw 0.5 per motor at 0.1 thrust: roll keeps 0.3 per motor and yaw goes to 0.
	EXPECT_EQ(allocate(2.f, 0.f, 2.f, -0.1f), Vector4f(0.f, 0.f, 0.6f, 0.6f));

	// Roll and pitch 0.3, yaw 0.1 per motor: roll must not come out reversed to keep yaw.
	EXPECT_EQ(allocate(1.2f, 1.2f, 0.4f, -0.1f), Vector4f(0.6f, 0.f, 0.f, 0.6f));
}

// Desaturating yaw may only shrink the commanded yaw toward zero. Run before roll/pitch on a
// limited budget, an unbounded pass would otherwise use yaw as a free axis to relieve a one-sided
// roll/pitch saturation: yaw nobody commanded, or yaw of the opposite sign.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, YawAirmodeNeverReversesOrInjectsYaw)
{
	const float torques[] = {-4.f, -2.f, -1.f, -0.4f, 0.f, 0.4f, 1.f, 2.f, 4.f};
	const float thrusts[] = {0.f, 0.05f, 0.1f, 0.3f, 0.5f, 0.7f, 0.9f, 1.f};

	for (const float lim : {0.2f, 0.5f, 1.f}) {
		setAirmodeParams(lim, true);

		for (const float roll : torques) {
			for (const float pitch : torques) {
				for (const float yaw : torques) {
					for (const float thrust : thrusts) {
						const Vector4f out = allocate(roll, pitch, yaw, -thrust);
						const float delivered_yaw = out(0) - out(1) + out(2) - out(3);
						const char *const where = "yaw outside [0, command]";

						EXPECT_GE(delivered_yaw, fminf(yaw, 0.f) - 1e-4f) << where << " LIM=" << lim << " roll=" << roll
								<< " pitch=" << pitch << " yaw=" << yaw << " thrust=" << thrust;
						EXPECT_LE(delivered_yaw, fmaxf(yaw, 0.f) + 1e-4f) << where << " LIM=" << lim << " roll=" << roll
								<< " pitch=" << pitch << " yaw=" << yaw << " thrust=" << thrust;
					}
				}
			}
		}
	}
}

// Mirroring the airframe about its x axis maps (roll, pitch, yaw) to (-roll, pitch, -yaw) and swaps
// motors 0<->3 and 1<->2, so mirrored commands must give mirrored outputs. A limit applied to the
// yaw axis breaks this: a negative yaw gain reduces positive yaw but increases negative yaw.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, AirmodeMirrorSymmetric)
{
	const float torques[] = {-4.f, -2.f, -1.f, 0.f, 1.f, 2.f, 4.f};
	const float thrusts[] = {0.05f, 0.1f, 0.3f, 0.5f, 0.7f, 0.9f};

	for (const float lim : {0.f, 0.2f, 0.5f, 1.f}) {
		for (const bool yaw_airmode : {false, true}) {
			setAirmodeParams(lim, yaw_airmode);
			int asymmetric = 0;

			for (const float roll : torques) {
				for (const float pitch : torques) {
					for (const float yaw : torques) {
						for (const float thrust : thrusts) {
							const Vector4f out = allocate(roll, pitch, yaw, -thrust);
							const Vector4f mirrored = allocate(-roll, pitch, -yaw, -thrust);

							if (!(out == Vector4f(mirrored(3), mirrored(2), mirrored(1), mirrored(0))) && (asymmetric++ == 0)) {
								ADD_FAILURE() << "first asymmetry at roll=" << roll << " pitch=" << pitch << " yaw=" << yaw
									      << " thrust=" << thrust;
							}
						}
					}
				}
			}

			EXPECT_EQ(asymmetric, 0) << "LIM=" << lim << " yaw airmode=" << yaw_airmode;
		}
	}
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, PreviousMixingTestsNoAirmode)
{
	setAirmode(0); // No airmode
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 1
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.100000f, 0.100000f, 0.100000f, 0.100000f)); // 2
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.450f), Vector4f(0.450000f, 0.450000f, 0.450000f, 0.450000f)); // 3
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.900f), Vector4f(0.900000f, 0.900000f, 0.900000f, 0.900000f)); // 4
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -1.000f), Vector4f(1.000000f, 1.000000f, 1.000000f, 1.000000f)); // 5
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 6
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.100f), Vector4f(0.112500f, 0.112500f, 0.087500f, 0.087500f)); // 7
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.450f), Vector4f(0.462500f, 0.462500f, 0.437500f, 0.437500f)); // 8
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.900f), Vector4f(0.912500f, 0.912500f, 0.887500f, 0.887500f)); // 9
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -1.000f), Vector4f(1.000000f, 1.000000f, 0.975000f, 0.975000f)); // 10
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 11
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.100f), Vector4f(0.075000f, 0.100000f, 0.125000f, 0.100000f)); // 12
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.450f), Vector4f(0.425000f, 0.450000f, 0.475000f, 0.450000f)); // 13
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.900f), Vector4f(0.875000f, 0.900000f, 0.925000f, 0.900000f)); // 14
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -1.000f), Vector4f(0.950000f, 0.975000f, 1.000000f, 0.975000f)); // 15
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 16
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.100f), Vector4f(0.093750f, 0.081250f, 0.093750f, 0.131250f)); // 17
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.450f), Vector4f(0.443750f, 0.431250f, 0.443750f, 0.481250f)); // 18
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.900f), Vector4f(0.893750f, 0.881250f, 0.893750f, 0.931250f)); // 19
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -1.000f), Vector4f(0.962500f, 0.950000f, 0.962500f, 1.000000f)); // 20
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 21
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.100f), Vector4f(0.143750f, 0.056250f, 0.043750f, 0.156250f)); // 22
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.450f), Vector4f(0.493750f, 0.406250f, 0.393750f, 0.506250f)); // 23
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.900f), Vector4f(0.943750f, 0.856250f, 0.843750f, 0.956250f)); // 24
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -1.000f), Vector4f(0.987500f, 0.900000f, 0.887500f, 1.000000f)); // 25
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 26
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.100f), Vector4f(0.085000f, 0.015000f, 0.160000f, 0.140000f)); // 27
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.450f), Vector4f(0.435000f, 0.365000f, 0.510000f, 0.490000f)); // 28
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.900f), Vector4f(0.885000f, 0.815000f, 0.960000f, 0.940000f)); // 29
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -1.000f), Vector4f(0.922500f, 0.852500f, 0.997500f, 0.977500f)); // 30
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 31
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.100f), Vector4f(0.146250f, 0.116250f, 0.073750f, 0.063750f)); // 32
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.450f), Vector4f(0.496250f, 0.466250f, 0.423750f, 0.413750f)); // 33
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.900f), Vector4f(0.946250f, 0.916250f, 0.873750f, 0.863750f)); // 34
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -1.000f), Vector4f(1.000000f, 0.970000f, 0.927500f, 0.917500f)); // 35
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 36
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.000000f, 0.000000f, 0.200000f, 0.200000f)); // 37
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.450f), Vector4f(0.200000f, 0.200000f, 0.700000f, 0.700000f)); // 38
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.900f), Vector4f(0.500000f, 0.500000f, 1.000000f, 1.000000f)); // 39
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -1.000f), Vector4f(0.500000f, 0.500000f, 1.000000f, 1.000000f)); // 40
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 41
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.100f), Vector4f(0.000000f, 0.200000f, 0.200000f, 0.000000f)); // 42
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.450f), Vector4f(0.200000f, 0.700000f, 0.700000f, 0.200000f)); // 43
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.900f), Vector4f(0.500000f, 1.000000f, 1.000000f, 0.500000f)); // 44
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -1.000f), Vector4f(0.500000f, 1.000000f, 1.000000f, 0.500000f)); // 45
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 46
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.100f), Vector4f(0.200000f, 0.000000f, 0.200000f, 0.000000f)); // 47
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.450f), Vector4f(0.700000f, 0.200000f, 0.700000f, 0.200000f)); // 48
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.900f), Vector4f(1.000000f, 0.500000f, 1.000000f, 0.500000f)); // 49
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -1.000f), Vector4f(1.000000f, 0.700000f, 1.000000f, 0.700000f)); // 50
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 51
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.100f), Vector4f(0.200000f, 0.000000f, 0.000000f, 0.200000f)); // 52
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.450f), Vector4f(0.100000f, 0.100000f, 0.000000f, 1.000000f)); // 53
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.900f), Vector4f(0.200000f, 0.000000f, 0.200000f, 1.000000f)); // 54
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -1.000f), Vector4f(0.200000f, 0.000000f, 0.200000f, 1.000000f)); // 55
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 56
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.100f), Vector4f(0.200000f, 0.000000f, 0.000000f, 0.200000f)); // 57
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.450f), Vector4f(0.900000f, 0.450000f, 0.000000f, 0.450000f)); // 58
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.900f), Vector4f(0.950000f, 0.600000f, 0.000000f, 0.550000f)); // 59
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -1.000f), Vector4f(0.950000f, 0.600000f, 0.000000f, 0.550000f)); // 60
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 61
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.100f), Vector4f(0.200000f, 0.000000f, 0.000000f, 0.200000f)); // 62
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.450f), Vector4f(0.900000f, 0.450000f, 0.000000f, 0.450000f)); // 63
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.900f), Vector4f(1.000000f, 0.550000f, 0.050000f, 0.500000f)); // 64
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -1.000f), Vector4f(1.000000f, 0.550000f, 0.050000f, 0.500000f)); // 65
}

// Rows tagged "yaw folded" (16, 30, 51, 52, 59, 60) differ from the historical multirotor-mixer
// values because the deferred-yaw path now folds yaw into the collective-thrust desaturation when
// that relieves the saturating actuator, yielding exactly the full roll/pitch/yaw-airmode
// allocation for the command: roll, pitch and yaw are delivered at least as well (never worse),
// while collective thrust may shift to make room for the yaw torque.
TEST_F(ControlAllocationSequentialDesaturationTestQuadX, PreviousMixingTestsAirmodeRP)
{
	setAirmode(1); // Roll and pitch airmode
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 1
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.100000f, 0.100000f, 0.100000f, 0.100000f)); // 2
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.450f), Vector4f(0.450000f, 0.450000f, 0.450000f, 0.450000f)); // 3
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.900f), Vector4f(0.900000f, 0.900000f, 0.900000f, 0.900000f)); // 4
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -1.000f), Vector4f(1.000000f, 1.000000f, 1.000000f, 1.000000f)); // 5
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.000f), Vector4f(0.025000f, 0.025000f, 0.000000f, 0.000000f)); // 6
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.100f), Vector4f(0.112500f, 0.112500f, 0.087500f, 0.087500f)); // 7
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.450f), Vector4f(0.462500f, 0.462500f, 0.437500f, 0.437500f)); // 8
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.900f), Vector4f(0.912500f, 0.912500f, 0.887500f, 0.887500f)); // 9
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -1.000f), Vector4f(1.000000f, 1.000000f, 0.975000f, 0.975000f)); // 10
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.000f), Vector4f(0.000000f, 0.025000f, 0.050000f, 0.025000f)); // 11
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.100f), Vector4f(0.075000f, 0.100000f, 0.125000f, 0.100000f)); // 12
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.450f), Vector4f(0.425000f, 0.450000f, 0.475000f, 0.450000f)); // 13
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.900f), Vector4f(0.875000f, 0.900000f, 0.925000f, 0.900000f)); // 14
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -1.000f), Vector4f(0.950000f, 0.975000f, 1.000000f, 0.975000f)); // 15
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.000f), Vector4f(0.012500f, 0.000000f, 0.012500f, 0.050000f)); // 16 yaw folded
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.100f), Vector4f(0.093750f, 0.081250f, 0.093750f, 0.131250f)); // 17
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.450f), Vector4f(0.443750f, 0.431250f, 0.443750f, 0.481250f)); // 18
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.900f), Vector4f(0.893750f, 0.881250f, 0.893750f, 0.931250f)); // 19
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -1.000f), Vector4f(0.962500f, 0.950000f, 0.962500f, 1.000000f)); // 20
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.000f), Vector4f(0.100000f, 0.000000f, 0.000000f, 0.100000f)); // 21
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.100f), Vector4f(0.143750f, 0.056250f, 0.043750f, 0.156250f)); // 22
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.450f), Vector4f(0.493750f, 0.406250f, 0.393750f, 0.506250f)); // 23
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.900f), Vector4f(0.943750f, 0.856250f, 0.843750f, 0.956250f)); // 24
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -1.000f), Vector4f(0.987500f, 0.900000f, 0.887500f, 1.000000f)); // 25
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.000f), Vector4f(0.025000f, 0.000000f, 0.100000f, 0.125000f)); // 26
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.100f), Vector4f(0.085000f, 0.015000f, 0.160000f, 0.140000f)); // 27
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.450f), Vector4f(0.435000f, 0.365000f, 0.510000f, 0.490000f)); // 28
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.900f), Vector4f(0.885000f, 0.815000f, 0.960000f, 0.940000f)); // 29
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -1.000f), Vector4f(0.925000f, 0.855000f, 1.000000f, 0.980000f)); // 30 yaw folded
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.000f), Vector4f(0.082500f, 0.052500f, 0.010000f, 0.000000f)); // 31
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.100f), Vector4f(0.146250f, 0.116250f, 0.073750f, 0.063750f)); // 32
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.450f), Vector4f(0.496250f, 0.466250f, 0.423750f, 0.413750f)); // 33
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.900f), Vector4f(0.946250f, 0.916250f, 0.873750f, 0.863750f)); // 34
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -1.000f), Vector4f(1.000000f, 0.970000f, 0.927500f, 0.917500f)); // 35
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.500000f, 0.500000f)); // 36
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.000000f, 0.000000f, 0.500000f, 0.500000f)); // 37
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.450f), Vector4f(0.200000f, 0.200000f, 0.700000f, 0.700000f)); // 38
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.900f), Vector4f(0.500000f, 0.500000f, 1.000000f, 1.000000f)); // 39
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -1.000f), Vector4f(0.500000f, 0.500000f, 1.000000f, 1.000000f)); // 40
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.500000f, 0.500000f, 0.000000f)); // 41
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.100f), Vector4f(0.000000f, 0.500000f, 0.500000f, 0.000000f)); // 42
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.450f), Vector4f(0.200000f, 0.700000f, 0.700000f, 0.200000f)); // 43
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.900f), Vector4f(0.500000f, 1.000000f, 1.000000f, 0.500000f)); // 44
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -1.000f), Vector4f(0.500000f, 1.000000f, 1.000000f, 0.500000f)); // 45
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 46
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.100f), Vector4f(0.200000f, 0.000000f, 0.200000f, 0.000000f)); // 47
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.450f), Vector4f(0.700000f, 0.200000f, 0.700000f, 0.200000f)); // 48
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.900f), Vector4f(1.000000f, 0.500000f, 1.000000f, 0.500000f)); // 49
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -1.000f), Vector4f(1.000000f, 0.700000f, 1.000000f, 0.700000f)); // 50
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 51 yaw folded
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.100f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 52 yaw folded
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.450f), Vector4f(0.200000f, 0.000000f, 0.200000f, 1.000000f)); // 53
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.900f), Vector4f(0.200000f, 0.000000f, 0.200000f, 1.000000f)); // 54
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -1.000f), Vector4f(0.200000f, 0.000000f, 0.200000f, 1.000000f)); // 55
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.000f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 56
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.100f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 57
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.450f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 58
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.900f), Vector4f(1.000000f, 1.000000f, 0.050000f, 0.950000f)); // 59 yaw folded
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -1.000f), Vector4f(1.000000f, 1.000000f, 0.050000f, 0.950000f)); // 60 yaw folded
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.000f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 61
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.100f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 62
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.450f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 63
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.900f), Vector4f(1.000000f, 0.550000f, 0.050000f, 0.500000f)); // 64
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -1.000f), Vector4f(1.000000f, 0.550000f, 0.050000f, 0.500000f)); // 65
}

TEST_F(ControlAllocationSequentialDesaturationTestQuadX, PreviousMixingTestsAirmodeRPY)
{
	setAirmode(2); // Roll, pitch and yaw airmode
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 0.000000f)); // 1
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.100000f, 0.100000f, 0.100000f, 0.100000f)); // 2
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.450f), Vector4f(0.450000f, 0.450000f, 0.450000f, 0.450000f)); // 3
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -0.900f), Vector4f(0.900000f, 0.900000f, 0.900000f, 0.900000f)); // 4
	EXPECT_EQ(allocate(0.000f, 0.000f, 0.000f, -1.000f), Vector4f(1.000000f, 1.000000f, 1.000000f, 1.000000f)); // 5
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.000f), Vector4f(0.025000f, 0.025000f, 0.000000f, 0.000000f)); // 6
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.100f), Vector4f(0.112500f, 0.112500f, 0.087500f, 0.087500f)); // 7
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.450f), Vector4f(0.462500f, 0.462500f, 0.437500f, 0.437500f)); // 8
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -0.900f), Vector4f(0.912500f, 0.912500f, 0.887500f, 0.887500f)); // 9
	EXPECT_EQ(allocate(-0.050f, 0.000f, 0.000f, -1.000f), Vector4f(1.000000f, 1.000000f, 0.975000f, 0.975000f)); // 10
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.000f), Vector4f(0.000000f, 0.025000f, 0.050000f, 0.025000f)); // 11
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.100f), Vector4f(0.075000f, 0.100000f, 0.125000f, 0.100000f)); // 12
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.450f), Vector4f(0.425000f, 0.450000f, 0.475000f, 0.450000f)); // 13
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -0.900f), Vector4f(0.875000f, 0.900000f, 0.925000f, 0.900000f)); // 14
	EXPECT_EQ(allocate(0.050f, -0.050f, 0.000f, -1.000f), Vector4f(0.950000f, 0.975000f, 1.000000f, 0.975000f)); // 15
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.000f), Vector4f(0.012500f, 0.000000f, 0.012500f, 0.050000f)); // 16
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.100f), Vector4f(0.093750f, 0.081250f, 0.093750f, 0.131250f)); // 17
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.450f), Vector4f(0.443750f, 0.431250f, 0.443750f, 0.481250f)); // 18
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -0.900f), Vector4f(0.893750f, 0.881250f, 0.893750f, 0.931250f)); // 19
	EXPECT_EQ(allocate(0.050f, 0.050f, -0.025f, -1.000f), Vector4f(0.962500f, 0.950000f, 0.962500f, 1.000000f)); // 20
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.000f), Vector4f(0.100000f, 0.012500f, 0.000000f, 0.112500f)); // 21
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.100f), Vector4f(0.143750f, 0.056250f, 0.043750f, 0.156250f)); // 22
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.450f), Vector4f(0.493750f, 0.406250f, 0.393750f, 0.506250f)); // 23
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -0.900f), Vector4f(0.943750f, 0.856250f, 0.843750f, 0.956250f)); // 24
	EXPECT_EQ(allocate(0.000f, 0.200f, -0.025f, -1.000f), Vector4f(0.987500f, 0.900000f, 0.887500f, 1.000000f)); // 25
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.000f), Vector4f(0.070000f, 0.000000f, 0.145000f, 0.125000f)); // 26
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.100f), Vector4f(0.085000f, 0.015000f, 0.160000f, 0.140000f)); // 27
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.450f), Vector4f(0.435000f, 0.365000f, 0.510000f, 0.490000f)); // 28
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -0.900f), Vector4f(0.885000f, 0.815000f, 0.960000f, 0.940000f)); // 29
	EXPECT_EQ(allocate(0.200f, 0.050f, 0.090f, -1.000f), Vector4f(0.925000f, 0.855000f, 1.000000f, 0.980000f)); // 30
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.000f), Vector4f(0.082500f, 0.052500f, 0.010000f, 0.000000f)); // 31
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.100f), Vector4f(0.146250f, 0.116250f, 0.073750f, 0.063750f)); // 32
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.450f), Vector4f(0.496250f, 0.466250f, 0.423750f, 0.413750f)); // 33
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -0.900f), Vector4f(0.946250f, 0.916250f, 0.873750f, 0.863750f)); // 34
	EXPECT_EQ(allocate(-0.125f, 0.020f, 0.040f, -1.000f), Vector4f(1.000000f, 0.970000f, 0.927500f, 0.917500f)); // 35
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.500000f, 0.500000f)); // 36
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.100f), Vector4f(0.000000f, 0.000000f, 0.500000f, 0.500000f)); // 37
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.450f), Vector4f(0.200000f, 0.200000f, 0.700000f, 0.700000f)); // 38
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -0.900f), Vector4f(0.500000f, 0.500000f, 1.000000f, 1.000000f)); // 39
	EXPECT_EQ(allocate(1.000f, 0.000f, 0.000f, -1.000f), Vector4f(0.500000f, 0.500000f, 1.000000f, 1.000000f)); // 40
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.000f), Vector4f(0.000000f, 0.500000f, 0.500000f, 0.000000f)); // 41
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.100f), Vector4f(0.000000f, 0.500000f, 0.500000f, 0.000000f)); // 42
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.450f), Vector4f(0.200000f, 0.700000f, 0.700000f, 0.200000f)); // 43
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -0.900f), Vector4f(0.500000f, 1.000000f, 1.000000f, 0.500000f)); // 44
	EXPECT_EQ(allocate(0.000f, -1.000f, 0.000f, -1.000f), Vector4f(0.500000f, 1.000000f, 1.000000f, 0.500000f)); // 45
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.000f), Vector4f(0.500000f, 0.000000f, 0.500000f, 0.000000f)); // 46
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.100f), Vector4f(0.500000f, 0.000000f, 0.500000f, 0.000000f)); // 47
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.450f), Vector4f(0.700000f, 0.200000f, 0.700000f, 0.200000f)); // 48
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -0.900f), Vector4f(1.000000f, 0.500000f, 1.000000f, 0.500000f)); // 49
	EXPECT_EQ(allocate(0.000f, 0.000f, 1.000f, -1.000f), Vector4f(1.000000f, 0.500000f, 1.000000f, 0.500000f)); // 50
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 51
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.100f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 52
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.450f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 53
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -0.900f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 54
	EXPECT_EQ(allocate(1.000f, 1.000f, -1.000f, -1.000f), Vector4f(0.000000f, 0.000000f, 0.000000f, 1.000000f)); // 55
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.000f), Vector4f(0.950000f, 0.950000f, 0.000000f, 0.900000f)); // 56
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.100f), Vector4f(0.950000f, 0.950000f, 0.000000f, 0.900000f)); // 57
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.450f), Vector4f(0.950000f, 0.950000f, 0.000000f, 0.900000f)); // 58
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -0.900f), Vector4f(1.000000f, 1.000000f, 0.050000f, 0.950000f)); // 59
	EXPECT_EQ(allocate(-1.000f, 0.900f, -0.900f, -1.000f), Vector4f(1.000000f, 1.000000f, 0.050000f, 0.950000f)); // 60
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.000f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 61
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.100f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 62
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.450f), Vector4f(0.950000f, 0.500000f, 0.000000f, 0.450000f)); // 63
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -0.900f), Vector4f(1.000000f, 0.550000f, 0.050000f, 0.500000f)); // 64
	EXPECT_EQ(allocate(-1.000f, 0.900f, 0.000f, -1.000f), Vector4f(1.000000f, 0.550000f, 0.050000f, 0.500000f)); // 65
}


// Hex fixture covers the QuadX blind spot: with 6 motors at 60-degree intervals the roll/pitch
// mix matrix is no longer ±0.25 like QuadX, so the 0.2 effectiveness-cutoff inside
// computeDesaturationGain interacts differently with the per-axis desat passes. Motor failure on
// a hex is also a strong motivation for limited airmode.
class ControlAllocationSequentialDesaturationTestHex : public ::testing::Test
{
public:
	static constexpr uint8_t NUM_ROTORS = 6;
	using HexVector = Vector<float, NUM_ROTORS>;

	ControlAllocationSequentialDesaturation _control_allocation;
	ActuatorEffectiveness::Configuration _config{};
	int _num_actuators{0};

	void SetUp() override
	{
		param_control_autosave(false);
		setAirmodeParams(0.f, false);

		// Hex geometry: 6 motors at 30, 90, 150, 210, 270, 330 degrees measured clockwise from
		// +X (forward) toward +Y (right), normalized unit arm length. Alternating moment ratios.
		// Numbering goes clockwise starting from the front-right sextant.
		const float positions[NUM_ROTORS][2] = {
			{ 0.866025f,  0.5f},     // M0  30 deg — front-right
			{ 0.f,        1.f},      // M1  90 deg — right
			{-0.866025f,  0.5f},     // M2 150 deg — back-right
			{-0.866025f, -0.5f},     // M3 210 deg — back-left
			{ 0.f,       -1.f},      // M4 270 deg — left
			{ 0.866025f, -0.5f},     // M5 330 deg — front-left
		};

		ActuatorEffectivenessRotors::Geometry hex_geometry{};
		hex_geometry.num_rotors = NUM_ROTORS;

		for (int i = 0; i < NUM_ROTORS; ++i) {
			hex_geometry.rotors[i].position = {positions[i][0], positions[i][1], 0.f};
			hex_geometry.rotors[i].moment_ratio = (i % 2 == 0) ? 1.f : -1.f;
			hex_geometry.rotors[i].axis = Vector3f(0.f, 0.f, -1.f);
			hex_geometry.rotors[i].thrust_coef = 1.f;
			hex_geometry.rotors[i].tilt_index = -1;
		}

		_num_actuators = ActuatorEffectivenessRotors::computeEffectivenessMatrix(
					 hex_geometry,
					 _config.effectiveness_matrices[0],
					 _config.num_actuators_matrix[0]);
		EXPECT_EQ(_num_actuators, NUM_ROTORS);
		_config.actuatorsAdded(ActuatorType::MOTORS, _num_actuators);

		applyEffectiveness();
	}

	void applyEffectiveness()
	{
		_control_allocation.setEffectivenessMatrix(
			_config.effectiveness_matrices[0],
			_config.trim[0],
			_config.linearization_point[0],
			_config.num_actuators_matrix[0],
			true);
	}

	// Model a detected motor failure the way ControlAllocator does: zero the rotor's column.
	void failMotor(int rotor)
	{
		for (int axis = 0; axis < ControlAllocation::NUM_AXES; ++axis) {
			_config.effectiveness_matrices[0](axis, rotor) = 0.f;
		}

		applyEffectiveness();
	}

	void setAirmodeParams(const float lim, const bool yaw)
	{
		const int32_t yaw_param = yaw ? 1 : 0;
		param_set(param_find("MC_AIRMODE_LIM"), &lim);
		param_set(param_find("MC_AIRMODE_YAW"), &yaw_param);
		_control_allocation.updateParameters();
	}

	HexVector allocate(float roll, float pitch, float yaw, float thrust)
	{
		Vector<float, ControlAllocation::NUM_AXES> control_setpoint{};
		control_setpoint(ControlAllocation::ControlAxis::ROLL) = roll;
		control_setpoint(ControlAllocation::ControlAxis::PITCH) = pitch;
		control_setpoint(ControlAllocation::ControlAxis::YAW) = yaw;
		control_setpoint(ControlAllocation::ControlAxis::THRUST_Z) = thrust;
		_control_allocation.setControlSetpoint(control_setpoint);
		_control_allocation.allocate();
		return HexVector(_control_allocation.getActuatorSetpoint().slice<NUM_ROTORS, 1>(0, 0));
	}
};

constexpr uint8_t ControlAllocationSequentialDesaturationTestHex::NUM_ROTORS;

TEST_F(ControlAllocationSequentialDesaturationTestHex, CollectiveThrust)
{
	HexVector expected;

	for (int i = 0; i < NUM_ROTORS; ++i) { expected(i) = 0.5f; }

	EXPECT_EQ(allocate(0.f, 0.f, 0.f, -0.5f), expected);
}

// Limited airmode is a graduated, bounded knob — not on/off. With one rotor stuck off (its
// effectiveness column zeroed, the way ControlAllocator handles a detected failure), a roll
// command at near-zero throttle is badly under-delivered without airmode. Raising the limit
// recovers the roll proportionally — spending collective thrust to do it — until the need is met,
// after which a larger limit is inert (so full airmode equals a sufficiently-large limited value).
// An intermediate limit buys partial authority at a bounded thrust cost, which neither 0 nor 1
// expresses. This is a single allocation snapshot, not a closed-loop result.
TEST_F(ControlAllocationSequentialDesaturationTestHex, AirmodeLimitGraduatesAuthority)
{
	using CA = ControlAllocation;
	failMotor(0);
	const auto &effectiveness = _config.effectiveness_matrices[0];

	// Roll torque actually delivered (effectiveness applied to the physically clamped motors).
	auto delivered_roll = [&](float lim) {
		setAirmodeParams(lim, false);
		const HexVector out = allocate(1.0f, 0.f, 0.f, -0.05f);
		float roll = 0.f;

		for (int i = 0; i < NUM_ROTORS; ++i) {
			roll += effectiveness(CA::ControlAxis::ROLL, i) * fmaxf(0.f, fminf(1.f, out(i)));
		}

		return roll;
	};

	// Without airmode the failed-rotor saturation costs most of the commanded roll (1.0).
	EXPECT_LT(delivered_roll(0.f), 0.2f);

	// The limit graduates the recovery: each step buys more authority — intermediate values matter.
	EXPECT_GT(delivered_roll(0.10f), delivered_roll(0.f));
	EXPECT_GT(delivered_roll(0.20f), delivered_roll(0.10f));
	EXPECT_GT(delivered_roll(0.30f), delivered_roll(0.20f));

	// An intermediate limit is genuinely partial — between off and full, not all-or-nothing.
	EXPECT_GT(delivered_roll(0.20f), 0.4f);
	EXPECT_LT(delivered_roll(0.20f), 0.8f);

	// Once the limit meets the need (~0.5), roll is fully restored and a larger limit is inert.
	EXPECT_NEAR(delivered_roll(0.50f), 1.0f, 1e-2f);
	EXPECT_FLOAT_EQ(delivered_roll(1.0f), delivered_roll(0.50f));
}

// Sweep MC_AIRMODE_LIM on a saturating-low input and verify monotonicity. Mirrors the QuadX
// monotonicity test but with the different mix matrix that hex produces, so a regression in the
// desaturator that depends on the 0.2 effectiveness cutoff would show here even if QuadX passes.
TEST_F(ControlAllocationSequentialDesaturationTestHex, AirmodeLimMonotonicity)
{
	const float sweep[] = {0.f, 0.05f, 0.10f, 0.15f, 1.f};
	HexVector prev{};

	for (size_t i = 0; i < sizeof(sweep) / sizeof(sweep[0]); ++i) {
		setAirmodeParams(sweep[i], false);
		HexVector out = allocate(0.5f, 0.f, 0.f, -0.1f);

		if (i > 0) {
			for (int m = 0; m < NUM_ROTORS; ++m) {
				EXPECT_GE(out(m), prev(m))
						<< "non-monotonic at MC_AIRMODE_LIM=" << sweep[i] << " motor=" << m;
			}
		}

		prev = out;
	}
}

// With yaw airmode off and saturating yaw input, the deferred-yaw path uses MINIMUM_YAW_MARGIN to
// retain some yaw authority near max thrust. Sanity check that the path works on hex (different
// yaw mix sign pattern than QuadX).
TEST_F(ControlAllocationSequentialDesaturationTestHex, YawAtFullThrustDisabledHasMargin)
{
	setAirmodeParams(0.f, false);
	const HexVector out = allocate(0.f, 0.f, 1.f, -1.f);

	// All motors clamped to [0, 1].
	for (int i = 0; i < NUM_ROTORS; ++i) {
		EXPECT_GE(out(i), 0.f) << "motor " << i << " below min";
		EXPECT_LE(out(i), 1.f) << "motor " << i << " above max";
	}

	// Yaw differential must be non-zero: at least one CW motor must differ from at least one
	// CCW motor. Otherwise yaw authority is dead and the 15% margin trick failed.
	float ccw_avg = 0.f; // moment_ratio = +1 → indices 0, 2, 4
	float cw_avg = 0.f;  // moment_ratio = -1 → indices 1, 3, 5

	for (int i = 0; i < NUM_ROTORS; i += 2) { ccw_avg += out(i); }

	for (int i = 1; i < NUM_ROTORS; i += 2) { cw_avg += out(i); }

	ccw_avg /= 3.f;
	cw_avg /= 3.f;
	EXPECT_GT(fabsf(ccw_avg - cw_avg), 0.05f) << "yaw authority lost at full thrust (ccw_avg="
			<< ccw_avg << " cw_avg=" << cw_avg << ")";
}

// Same mirror symmetry as the QuadX test, with the hex mix: about the x axis M0<->M5, M1<->M4 and
// M2<->M3, each pair spinning in opposite directions.
TEST_F(ControlAllocationSequentialDesaturationTestHex, AirmodeMirrorSymmetric)
{
	const float torques[] = {-4.f, -2.f, -1.f, -0.5f, 0.f, 0.5f, 1.f, 2.f, 4.f};
	const float thrusts[] = {0.05f, 0.1f, 0.3f, 0.5f, 0.7f, 0.9f};

	for (const float lim : {0.f, 0.2f, 0.5f, 1.f}) {
		for (const bool yaw_airmode : {false, true}) {
			setAirmodeParams(lim, yaw_airmode);
			int asymmetric = 0;

			for (const float roll : torques) {
				for (const float pitch : torques) {
					for (const float yaw : torques) {
						for (const float thrust : thrusts) {
							const HexVector out = allocate(roll, pitch, yaw, -thrust);
							const HexVector mirrored = allocate(-roll, pitch, -yaw, -thrust);
							HexVector expected;

							for (int i = 0; i < NUM_ROTORS; ++i) { expected(i) = mirrored(NUM_ROTORS - 1 - i); }

							if (!(out == expected) && (asymmetric++ == 0)) {
								ADD_FAILURE() << "first asymmetry at roll=" << roll << " pitch=" << pitch << " yaw=" << yaw
									      << " thrust=" << thrust;
							}
						}
					}
				}
			}

			EXPECT_EQ(asymmetric, 0) << "LIM=" << lim << " yaw airmode=" << yaw_airmode;
		}
	}
}
