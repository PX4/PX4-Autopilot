// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "TurtleMixing.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <iterator>
#include <limits>

namespace
{
TurtleMixing::Geometry quad()
{
	TurtleMixing::Geometry geometry{};
	geometry.num_rotors = 4;
	geometry.rotors[0].position = {1.f, 1.f, 0.f};
	geometry.rotors[1].position = {-1.f, -1.f, 0.f};
	geometry.rotors[2].position = {1.f, -1.f, 0.f};
	geometry.rotors[3].position = {-1.f, 1.f, 0.f};

	for (unsigned i = 0; i < TurtleMixing::MOTOR_COUNT; ++i) {
		geometry.rotors[i].axis = {0.f, 0.f, -1.f};
		geometry.rotors[i].thrust_coef = 1.f;
		geometry.rotors[i].tilt_index = -1;
	}

	return geometry;
}

void expectStopped(const float (&controls)[actuator_motors_s::NUM_CONTROLS])
{
	for (float value : controls) {
		EXPECT_TRUE(std::isnan(value));
	}
}
}

TEST(TurtleMixing, FourPairsAndFourCornersInEveryMotorOrder)
{
	const auto source = quad();
	int order[] {0, 1, 2, 3};

	do {
		auto geometry = source;

		for (unsigned i = 0; i < TurtleMixing::MOTOR_COUNT; ++i) {
			geometry.rotors[i] = source.rotors[order[i]];
		}

		TurtleMixing mixer;
		mixer.configure(&geometry);
		ASSERT_TRUE(mixer.geometryValid());

		for (int roll = -1; roll <= 1; ++roll) {
			for (int pitch = -1; pitch <= 1; ++pitch) {
				if (roll == 0 && pitch == 0) {
					continue;
				}

				float controls[actuator_motors_s::NUM_CONTROLS];
				ASSERT_TRUE(mixer.mix(roll, pitch, 0.4f, controls));
				unsigned selected = 0;

				for (unsigned i = 0; i < TurtleMixing::MOTOR_COUNT; ++i) {
					const bool expected = (roll == 0 || roll * geometry.rotors[i].position(1) > 0.f)
							      && (pitch == 0 || pitch * geometry.rotors[i].position(0) > 0.f);

					if (expected) {
						EXPECT_FLOAT_EQ(controls[i], 0.4f);
						++selected;

					} else {
						EXPECT_TRUE(std::isnan(controls[i]));
					}
				}

				EXPECT_EQ(selected, roll == 0 || pitch == 0 ? 2u : 1u);

				for (unsigned i = TurtleMixing::MOTOR_COUNT; i < actuator_motors_s::NUM_CONTROLS; ++i) {
					EXPECT_TRUE(std::isnan(controls[i]));
				}
			}
		}
	} while (std::next_permutation(std::begin(order), std::end(order)));
}

TEST(TurtleMixing, CenteredSticksStopAllMotors)
{
	const auto geometry = quad();
	TurtleMixing mixer;
	mixer.configure(&geometry);
	float controls[actuator_motors_s::NUM_CONTROLS];
	ASSERT_TRUE(mixer.mix(0.f, 0.f, 0.4f, controls));
	expectStopped(controls);
	ASSERT_TRUE(mixer.mix(TurtleMixing::STICK_DEADBAND, -TurtleMixing::STICK_DEADBAND, 0.4f, controls));
	expectStopped(controls);
}

TEST(TurtleMixing, StickMagnitudeControlsBoundedThrust)
{
	const auto geometry = quad();
	TurtleMixing mixer;
	mixer.configure(&geometry);
	float controls[actuator_motors_s::NUM_CONTROLS];
	ASSERT_TRUE(mixer.mix(0.6f, 0.01f, 0.4f, controls));
	EXPECT_NEAR(controls[0], 0.2f, 1e-6f);
	EXPECT_NEAR(controls[3], 0.2f, 1e-6f);
	EXPECT_TRUE(std::isnan(controls[1]));
	EXPECT_TRUE(std::isnan(controls[2]));
}

TEST(TurtleMixing, InvalidInputStopsAllMotors)
{
	const float invalid[] {NAN, std::numeric_limits<float>::infinity(), -1.01f, 1.01f};
	const auto geometry = quad();
	TurtleMixing mixer;
	mixer.configure(&geometry);
	float controls[actuator_motors_s::NUM_CONTROLS];

	for (float value : invalid) {
		EXPECT_FALSE(mixer.mix(value, 1.f, 0.4f, controls));
		expectStopped(controls);
		EXPECT_FALSE(mixer.mix(1.f, value, 0.4f, controls));
		expectStopped(controls);
	}

	for (float value : {NAN, -1.f, 0.f, 1.01f, INFINITY}) {
		EXPECT_FALSE(mixer.mix(1.f, 1.f, value, controls));
		expectStopped(controls);
	}
}

TEST(TurtleMixing, RejectsUnsupportedGeometry)
{
	auto geometry = quad();
	TurtleMixing mixer;
	mixer.configure(&geometry);
	EXPECT_TRUE(mixer.geometryValid());
	geometry.num_rotors = 6;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].position = geometry.rotors[1].position;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].position(0) = 0.f;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].axis(2) = 1.f;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].axis(0) = 0.5f;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].tilt_index = 0;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].thrust_coef = 0.f;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
}

TEST(TurtleMixing, RejectsNonFiniteGeometry)
{
	TurtleMixing mixer;

	for (unsigned axis = 0; axis < 3; ++axis) {
		auto geometry = quad();
		geometry.rotors[0].position(axis) = NAN;
		mixer.configure(&geometry);
		EXPECT_FALSE(mixer.geometryValid());
		geometry = quad();
		geometry.rotors[0].axis(axis) = NAN;
		mixer.configure(&geometry);
		EXPECT_FALSE(mixer.geometryValid());
	}

	auto geometry = quad();
	geometry.rotors[0].thrust_coef = NAN;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	geometry = quad();
	geometry.rotors[0].moment_ratio = NAN;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
}

TEST(TurtleMixing, GeometryChangesTakeEffectOnReconfiguration)
{
	auto geometry = quad();
	TurtleMixing mixer;
	float controls[actuator_motors_s::NUM_CONTROLS];
	mixer.configure(&geometry);
	std::swap(geometry.rotors[0], geometry.rotors[1]);

	ASSERT_TRUE(mixer.mix(1.f, 1.f, 0.4f, controls));
	EXPECT_FLOAT_EQ(controls[0], 0.4f);
	EXPECT_TRUE(std::isnan(controls[1]));

	mixer.configure(&geometry);
	ASSERT_TRUE(mixer.mix(1.f, 1.f, 0.4f, controls));
	EXPECT_TRUE(std::isnan(controls[0]));
	EXPECT_FLOAT_EQ(controls[1], 0.4f);

	geometry.rotors[0].position = geometry.rotors[1].position;
	mixer.configure(&geometry);
	EXPECT_FALSE(mixer.geometryValid());
	EXPECT_FALSE(mixer.mix(1.f, 1.f, 0.4f, controls));
	expectStopped(controls);
}
