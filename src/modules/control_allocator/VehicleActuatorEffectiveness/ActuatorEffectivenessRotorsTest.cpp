/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessRotorsTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include "ActuatorEffectivenessRotors.hpp"

using namespace matrix;

TEST(ActuatorEffectivenessRotors, QuadrotorX)
{
	ActuatorEffectivenessRotors::Geometry geometry = {};
	geometry.rotors[0].position = {1.f, 1.f, 0.f};
	geometry.rotors[0].axis = {0.f, 0.f, -1.f};
	geometry.rotors[0].thrust_coef = 1.0f;
	geometry.rotors[0].moment_ratio = 0.05f;

	geometry.rotors[1].position = {-1.f, -1.f, 0.f};
	geometry.rotors[1].axis = {0.f, 0.f, -1.f};
	geometry.rotors[1].thrust_coef = 1.0f;
	geometry.rotors[1].moment_ratio = 0.05f;

	geometry.rotors[2].position = {1.f, -1.f, 0.f};
	geometry.rotors[2].axis = {0.f, 0.f, -1.f};
	geometry.rotors[2].thrust_coef = 1.0f;
	geometry.rotors[2].moment_ratio = -0.05f;

	geometry.rotors[3].position = {-1.f, 1.f, 0.f};
	geometry.rotors[3].axis = {0.f, 0.f, -1.f};
	geometry.rotors[3].thrust_coef = 1.0f;
	geometry.rotors[3].moment_ratio = -0.05f;

	geometry.num_rotors = 4;

	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{-1.0f,   1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 1.0f,  -1.0f,   1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.05f,  0.05f, -0.05f, -0.05f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-1.0f,  -1.0f,  -1.0f,  -1.0f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	ActuatorEffectiveness::EffectivenessMatrix effectiveness_expected(expected);

	EXPECT_EQ(effectiveness, effectiveness_expected);
}

TEST(ActuatorEffectivenessRotors, HexarotorX)
{
	ActuatorEffectivenessRotors::Geometry geometry = {};
	geometry.rotors[0].position = {0.f, .5f, 0.f};
	geometry.rotors[0].axis = {0.f, 0.f, -1.f};
	geometry.rotors[0].thrust_coef = 1.0f;
	geometry.rotors[0].moment_ratio = -0.05f;

	geometry.rotors[1].position = {0.f, -.5f, 0.f};
	geometry.rotors[1].axis = {0.f, 0.f, -1.f};
	geometry.rotors[1].thrust_coef = 1.0f;
	geometry.rotors[1].moment_ratio = 0.05f;

	geometry.rotors[2].position = {.43f, -.25f, 0.f};
	geometry.rotors[2].axis = {0.f, 0.f, -1.f};
	geometry.rotors[2].thrust_coef = 1.0f;
	geometry.rotors[2].moment_ratio = -0.05f;

	geometry.rotors[3].position = {-.43f, .25f, 0.f};
	geometry.rotors[3].axis = {0.f, 0.f, -1.f};
	geometry.rotors[3].thrust_coef = 1.0f;
	geometry.rotors[3].moment_ratio = 0.05f;

	geometry.rotors[4].position = {.43f, .25f, 0.f};
	geometry.rotors[4].axis = {0.f, 0.f, -1.f};
	geometry.rotors[4].thrust_coef = 1.0f;
	geometry.rotors[4].moment_ratio = 0.05f;

	geometry.rotors[5].position = {-.43f, -.25f, 0.f};
	geometry.rotors[5].axis = {0.f, 0.f, -1.f};
	geometry.rotors[5].thrust_coef = 1.0f;
	geometry.rotors[5].moment_ratio = -0.05f;

	geometry.num_rotors = 6;

	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry, effectiveness);

	const float expected[ActuatorEffectiveness::NUM_AXES][ActuatorEffectiveness::NUM_ACTUATORS] = {
		{-0.5f,  0.5f,   0.25f, -0.25f, -0.25f,  0.25f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,   0.f,    0.43f, -0.43f,  0.43f, -0.43f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-0.05f, 0.05f, -0.05f,  0.05f,  0.05f, -0.05f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,   0.f,    0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,   0.f,    0.f,    0.f,    0.f,    0.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-1.f,  -1.f,   -1.f,   -1.f,   -1.f,   -1.f,   0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	ActuatorEffectiveness::EffectivenessMatrix effectiveness_expected(expected);

	EXPECT_EQ(effectiveness, effectiveness_expected);
}

TEST(ActuatorEffectivenessRotors, Tilt)
{
	Vector3f axis_expected{0.f, 0.f, -1.f};
	Vector3f axis = ActuatorEffectivenessRotors::tiltedAxis(0.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{1.f, 0.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(M_PI_F / 2.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{1.f / sqrtf(2.f), 0.f, -1.f / sqrtf(2.f)};
	axis = ActuatorEffectivenessRotors::tiltedAxis(M_PI_F / 2.f / 2.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{-1.f, 0.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(-M_PI_F / 2.f, 0.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{0.f, 0.f, -1.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(0.f, M_PI_F / 2.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{0.f, 1.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(M_PI_F / 2.f, M_PI_F / 2.f);
	EXPECT_EQ(axis, axis_expected);

	axis_expected = Vector3f{0.f, -1.f, 0.f};
	axis = ActuatorEffectivenessRotors::tiltedAxis(-M_PI_F / 2.f, M_PI_F / 2.f);
	EXPECT_EQ(axis, axis_expected);
}


TEST(ActuatorEffectivenessRotors, isAlignedWithCoordinateAxis)
{
	// Exactly along z axis
	Vector3f vec = {0.f, 0.f, 1.f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_TRUE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Approximately along x axis (atan(0.1) = 5.7 deg, below 10 deg tolerance)
	vec = Vector3f{1.f, 0.f, -0.1f};
	EXPECT_TRUE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Approximately along x axis (atan(0.2) = 11.3 deg, above 10 deg tolerance)
	vec = Vector3f{1.f, 0.f, -0.2f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Approximately along y axis, with both other axes slightly off
	// (atan(0.1 sqrt(2)) = 8.05 deg, below 10 deg tolerance)
	vec = Vector3f{0.1f, 1.0f, -0.1f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_TRUE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Same but with larger offset: atan(0.2 sqrt(2)) = 15.79 deg
	vec = Vector3f{0.2f, 1.0f, -0.2f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

	// Random vector far from any axes
	vec = Vector3f{-100.f, -100.f, 100.f};
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 0));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 1));
	EXPECT_FALSE(ActuatorEffectivenessRotors::isAlignedWithCoordinateAxis(vec, 2));

}
