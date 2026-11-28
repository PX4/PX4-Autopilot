/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include <initializer_list>
#include <ActuatorEffectivenessRotors.hpp>
#include "ControlAllocationPseudoInverse.hpp"
#include "ControlAllocationSequentialDesaturation.hpp"

using namespace matrix;
using ActuatorVector = ControlAllocation::ActuatorVector;
using ControlVector = Vector<float, ControlAllocation::NUM_AXES>;

class ControlAllocationSparseAxesTest : public ::testing::Test
{
protected:
	static constexpr float kTolerance = 2e-5f;
	static constexpr float kCollective = .4f;
	static constexpr float kTorque = .1f;

	void SetUp() override
	{
		param_control_autosave(false);
		_airmode = param_find("MC_AIRMODE");
		ASSERT_NE(_airmode, PARAM_INVALID);
		ASSERT_EQ(param_get(_airmode, &_previous_airmode), 0);
		const int32_t disabled = 0;
		ASSERT_EQ(param_set(_airmode, &disabled), 0);
	}

	void TearDown() override
	{
		if (_airmode != PARAM_INVALID) {
			EXPECT_EQ(param_set(_airmode, &_previous_airmode), 0);
		}
	}

	static ActuatorEffectivenessRotors::Geometry duo(float coefficient, bool pitch_only = false)
	{
		ActuatorEffectivenessRotors::Geometry geometry{};
		geometry.num_rotors = 2;
		geometry.yaw_by_differential_thrust_disabled = true;
		geometry.rotors[0].position = pitch_only ? Vector3f(1.f, 0.f, 0.f) : Vector3f(0.f, 1.f, 0.f);
		geometry.rotors[1].position = -geometry.rotors[0].position;

		for (int i = 0; i < geometry.num_rotors; ++i) {
			geometry.rotors[i].axis = Vector3f(0.f, 0.f, -1.f);
			geometry.rotors[i].thrust_coef = coefficient;
			geometry.rotors[i].tilt_index = -1;
		}

		return geometry;
	}

	void loadGeometry(ControlAllocationPseudoInverse &allocator,
			  const ActuatorEffectivenessRotors::Geometry &geometry, bool update_normalization)
	{
		ActuatorEffectiveness::Configuration configuration{};
		const int count = ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry,
				  configuration.effectiveness_matrices[0]);
		EXPECT_EQ(count, geometry.num_rotors);
		configuration.actuatorsAdded(ActuatorType::MOTORS, count);
		allocator.setEffectivenessMatrix(configuration.effectiveness_matrices[0], configuration.trim[0],
						 configuration.linearization_point[0], configuration.num_actuators_matrix[0], update_normalization);
	}

	void configure(ControlAllocationPseudoInverse &allocator,
		       const ActuatorEffectivenessRotors::Geometry &geometry, bool normalize = true, bool metric = false)
	{
		allocator.updateParameters();
		allocator.setNormalizeRPY(normalize);
		allocator.setMetricAllocation(metric);
		ActuatorVector maximum;
		maximum.setAll(1.f);
		allocator.setActuatorMin(ActuatorVector());
		allocator.setActuatorMax(maximum);
		loadGeometry(allocator, geometry, true);
	}

	ActuatorVector allocate(const ActuatorEffectivenessRotors::Geometry &geometry, const ControlVector &control,
				bool sequential, bool normalize = true)
	{
		ControlAllocationPseudoInverse pseudo_inverse;
		ControlAllocationSequentialDesaturation sequential_desaturation;
		ControlAllocationPseudoInverse &allocator = sequential ? sequential_desaturation : pseudo_inverse;
		configure(allocator, geometry, normalize);
		allocator.setControlSetpoint(control);
		allocator.allocate();
		const ActuatorVector result = allocator.getActuatorSetpoint();

		for (int i = 0; i < ControlAllocation::NUM_ACTUATORS; ++i) {
			EXPECT_TRUE(PX4_ISFINITE(result(i)));
			EXPECT_GE(result(i), -kTolerance);
			EXPECT_LE(result(i), 1.f + kTolerance);

			if (i >= geometry.num_rotors) {
				EXPECT_NEAR(result(i), 0.f, kTolerance);
			}
		}

		// Check the output before clipping so saturation cannot hide a normalization error.
		allocator.clipActuatorSetpoint();
		const ActuatorVector difference = allocator.getActuatorSetpoint() - result;
		EXPECT_LT(difference.norm(), kTolerance);
		return result;
	}

	static ControlVector demand(float roll = 0.f, float pitch = 0.f, float yaw = 0.f)
	{
		ControlVector control{};
		control(ControlAllocation::ROLL) = roll;
		control(ControlAllocation::PITCH) = pitch;
		control(ControlAllocation::YAW) = yaw;
		control(ControlAllocation::THRUST_Z) = -kCollective;
		return control;
	}

private:
	param_t _airmode{PARAM_INVALID};
	int32_t _previous_airmode{0};
};

TEST_F(ControlAllocationSparseAxesTest, RollOnlyCommonCoefficientInvariant)
{
	for (bool sequential : {false, true}) {
		for (float coefficient : {1.f, 2.f, 6.5f}) {
			for (float torque : {-kTorque, kTorque}) {
				SCOPED_TRACE(::testing::Message() << sequential << ", CT=" << coefficient << ", torque=" << torque);
				const ActuatorVector result = allocate(duo(coefficient), demand(torque), sequential);
				EXPECT_NEAR(result(1) - result(0), sqrtf(2.f) * torque, kTolerance);
				EXPECT_NEAR((result(0) + result(1)) * .5f, kCollective, kTolerance);
			}
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, PitchOnlyCommonCoefficientInvariant)
{
	for (bool sequential : {false, true}) {
		for (float coefficient : {1.f, 2.f, 6.5f}) {
			for (float torque : {-kTorque, kTorque}) {
				SCOPED_TRACE(::testing::Message() << sequential << ", CT=" << coefficient << ", torque=" << torque);
				const ActuatorVector result = allocate(duo(coefficient, true), demand(0.f, torque), sequential);
				EXPECT_NEAR(result(0) - result(1), sqrtf(2.f) * torque, kTolerance);
				EXPECT_NEAR((result(0) + result(1)) * .5f, kCollective, kTolerance);
			}
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, FullRankQuadKeepsSharedScale)
{
	for (bool sequential : {false, true}) {
		for (float coefficient : {1.f, 2.f, 6.5f}) {
			SCOPED_TRACE(::testing::Message() << sequential << ", CT=" << coefficient);
			ActuatorEffectivenessRotors::Geometry geometry{};
			geometry.num_rotors = 4;
			geometry.rotors[0].position = Vector3f(1.f, 1.f, 0.f);
			geometry.rotors[1].position = Vector3f(-1.f, 1.f, 0.f);
			geometry.rotors[2].position = Vector3f(-1.f, -1.f, 0.f);
			geometry.rotors[3].position = Vector3f(1.f, -1.f, 0.f);

			for (int i = 0; i < geometry.num_rotors; ++i) {
				geometry.rotors[i].axis = Vector3f(0.f, 0.f, -1.f);
				geometry.rotors[i].thrust_coef = coefficient;
				geometry.rotors[i].moment_ratio = i % 2 == 0 ? 1.f : -1.f;
				geometry.rotors[i].tilt_index = -1;
			}

			const ActuatorVector result = allocate(geometry, demand(.05f, .04f, .02f), sequential);
			const float rp_scale = 1.f / sqrtf(2.f);
			EXPECT_NEAR(result(0), kCollective + rp_scale * (-.05f + .04f) + .02f, kTolerance);
			EXPECT_NEAR(result(1), kCollective + rp_scale * (-.05f - .04f) - .02f, kTolerance);
			EXPECT_NEAR(result(2), kCollective + rp_scale * (.05f - .04f) + .02f, kTolerance);
			EXPECT_NEAR(result(3), kCollective + rp_scale * (.05f + .04f) - .02f, kTolerance);
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, DisabledNormalizationKeepsInverseCoefficient)
{
	for (bool sequential : {false, true}) {
		for (bool pitch_only : {false, true}) {
			for (float coefficient : {1.f, 2.f, 6.5f}) {
				SCOPED_TRACE(::testing::Message() << sequential << ", pitch=" << pitch_only << ", CT=" << coefficient);
				const ActuatorVector result = allocate(duo(coefficient, pitch_only),
								       demand(pitch_only ? 0.f : kTorque, pitch_only ? kTorque : 0.f), sequential, false);
				const float differential = pitch_only ? result(0) - result(1) : result(1) - result(0);
				EXPECT_NEAR(differential, kTorque / coefficient, kTolerance);
				EXPECT_NEAR((result(0) + result(1)) * .5f, kCollective, kTolerance);
			}
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, NoRollPitchKeepsCollectiveAndFiniteOutputs)
{
	for (bool sequential : {false, true}) {
		for (float coefficient : {1.f, 2.f, 6.5f}) {
			ActuatorEffectivenessRotors::Geometry geometry = duo(coefficient);
			geometry.rotors[0].position.zero();
			geometry.rotors[1].position.zero();
			const ActuatorVector result = allocate(geometry, demand(kTorque, kTorque), sequential);
			EXPECT_NEAR(result(0), kCollective, kTolerance);
			EXPECT_NEAR(result(1), kCollective, kTolerance);
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, AllZeroEffectivenessStaysFiniteAndUnallocated)
{
	for (bool sequential : {false, true}) {
		const ActuatorVector result = allocate(duo(0.f), demand(kTorque, kTorque, kTorque), sequential);
		EXPECT_NEAR(result.norm(), 0.f, kTolerance);
	}
}

TEST_F(ControlAllocationSparseAxesTest, YawOnlyKeepsIndependentNormalization)
{
	for (bool sequential : {false, true}) {
		for (float coefficient : {1.f, 2.f, 6.5f}) {
			ActuatorEffectivenessRotors::Geometry geometry = duo(coefficient);
			geometry.yaw_by_differential_thrust_disabled = false;
			geometry.rotors[0].position.zero();
			geometry.rotors[1].position.zero();
			geometry.rotors[0].moment_ratio = 1.f;
			geometry.rotors[1].moment_ratio = -1.f;
			const ActuatorVector result = allocate(geometry, demand(0.f, 0.f, kTorque), sequential);
			EXPECT_NEAR(result(0), kCollective + kTorque, kTolerance);
			EXPECT_NEAR(result(1), kCollective - kTorque, kTolerance);
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, SignedSaturationAndDesaturationStayFiniteAndBounded)
{
	for (bool sequential : {false, true}) {
		for (bool pitch_only : {false, true}) {
			for (float torque : {-1.f, 1.f}) {
				SCOPED_TRACE(::testing::Message() << sequential << ", pitch=" << pitch_only << ", torque=" << torque);
				ControlAllocationPseudoInverse pseudo_inverse;
				ControlAllocationSequentialDesaturation sequential_desaturation;
				ControlAllocationPseudoInverse &allocator = sequential ? sequential_desaturation : pseudo_inverse;
				configure(allocator, duo(1.f, pitch_only));
				allocator.setControlSetpoint(demand(pitch_only ? 0.f : torque, pitch_only ? torque : 0.f));
				allocator.allocate();
				const ActuatorVector before_clip = allocator.getActuatorSetpoint();

				if (sequential) {
					for (int i = 0; i < 2; ++i) {
						EXPECT_GE(before_clip(i), -kTolerance);
						EXPECT_LE(before_clip(i), 1.f + kTolerance);
					}

				} else {
					// Unit torque at this collective exceeds headroom; PseudoInverse requires clipping.
					EXPECT_TRUE(before_clip(0) < 0.f || before_clip(1) < 0.f || before_clip(0) > 1.f || before_clip(1) > 1.f);
				}

				allocator.clipActuatorSetpoint();
				const ActuatorVector result = allocator.getActuatorSetpoint();

				for (int i = 0; i < ControlAllocation::NUM_ACTUATORS; ++i) {
					EXPECT_TRUE(PX4_ISFINITE(result(i)));
					EXPECT_GE(result(i), 0.f);
					EXPECT_LE(result(i), 1.f);
				}

				const float differential = pitch_only ? result(0) - result(1) : result(1) - result(0);
				EXPECT_GT(differential * torque, 0.f);
				const ControlVector allocated = allocator.getAllocatedControl();
				EXPECT_LE(fabsf(allocated(pitch_only ? ControlAllocation::PITCH : ControlAllocation::ROLL)), 1.f + kTolerance);
			}
		}
	}
}

TEST_F(ControlAllocationSparseAxesTest, NoRenormalizationPreservesCapturedScale)
{
	for (bool sequential : {false, true}) {
		ControlAllocationPseudoInverse pseudo_inverse;
		ControlAllocationSequentialDesaturation sequential_desaturation;
		ControlAllocationPseudoInverse &allocator = sequential ? sequential_desaturation : pseudo_inverse;
		configure(allocator, duo(1.f));
		allocator.setControlSetpoint(demand(kTorque));
		allocator.allocate();
		const ActuatorVector initial = allocator.getActuatorSetpoint();
		loadGeometry(allocator, duo(2.f), false);
		allocator.allocate();
		const ActuatorVector frozen_scale = allocator.getActuatorSetpoint();

		for (int i = 0; i < 2; ++i) {
			EXPECT_NEAR(frozen_scale(i), initial(i) * .5f, kTolerance);
		}

		const ControlVector allocated = allocator.getAllocatedControl();
		EXPECT_NEAR(allocated(ControlAllocation::ROLL), kTorque, kTolerance);
		EXPECT_NEAR(allocated(ControlAllocation::THRUST_Z), -kCollective, kTolerance);
		loadGeometry(allocator, duo(1.f), true);
		allocator.allocate();
		const ActuatorVector recovered = allocator.getActuatorSetpoint();
		const ActuatorVector difference = recovered - initial;
		EXPECT_LT(difference.norm(), kTolerance);
	}
}

TEST_F(ControlAllocationSparseAxesTest, ActuatorFailureFreezesNormalizationScale)
{
	for (bool sequential : {false, true}) {
		ControlAllocationPseudoInverse pseudo_inverse;
		ControlAllocationSequentialDesaturation sequential_desaturation;
		ControlAllocationPseudoInverse &allocator = sequential ? sequential_desaturation : pseudo_inverse;
		configure(allocator, duo(1.f));
		allocator.setControlSetpoint(demand(kTorque));
		allocator.allocate();
		const ActuatorVector initial = allocator.getActuatorSetpoint();
		allocator.setHadActuatorFailure(true);
		loadGeometry(allocator, duo(2.f), true);
		allocator.allocate();
		const ActuatorVector frozen_scale = allocator.getActuatorSetpoint();

		for (int i = 0; i < 2; ++i) {
			EXPECT_NEAR(frozen_scale(i), initial(i) * .5f, kTolerance);
		}

		const ControlVector allocated = allocator.getAllocatedControl();
		EXPECT_NEAR(allocated(ControlAllocation::ROLL), kTorque, kTolerance);
		EXPECT_NEAR(allocated(ControlAllocation::THRUST_Z), -kCollective, kTolerance);
		allocator.setHadActuatorFailure(false);
		loadGeometry(allocator, duo(1.f), true);
		allocator.allocate();
		const ActuatorVector difference = allocator.getActuatorSetpoint() - initial;
		EXPECT_LT(difference.norm(), kTolerance);
	}
}

TEST_F(ControlAllocationSparseAxesTest, MetricAllocationKeepsAnalyticInverseCoefficient)
{
	for (bool sequential : {false, true}) {
		for (float coefficient : {1.f, 2.f, 6.5f}) {
			for (float torque : {-kTorque, kTorque}) {
				SCOPED_TRACE(::testing::Message() << sequential << ", CT=" << coefficient << ", torque=" << torque);
				ControlAllocationPseudoInverse pseudo_inverse;
				ControlAllocationSequentialDesaturation sequential_desaturation;
				ControlAllocationPseudoInverse &allocator = sequential ? sequential_desaturation : pseudo_inverse;
				configure(allocator, duo(coefficient), true, true);
				allocator.setControlSetpoint(demand(torque));
				allocator.allocate();
				const ActuatorVector result = allocator.getActuatorSetpoint();
				EXPECT_NEAR(result(0), (kCollective - torque) / (2.f * coefficient), kTolerance);
				EXPECT_NEAR(result(1), (kCollective + torque) / (2.f * coefficient), kTolerance);
				const ControlVector allocated = allocator.getAllocatedControl();
				EXPECT_NEAR(allocated(ControlAllocation::ROLL), torque, kTolerance);
				EXPECT_NEAR(allocated(ControlAllocation::THRUST_Z), -kCollective, kTolerance);
			}
		}
	}
}
