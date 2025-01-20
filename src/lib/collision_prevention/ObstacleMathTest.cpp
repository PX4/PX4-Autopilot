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
#include <matrix/math.hpp>
#include "ObstacleMath.hpp"

using namespace matrix;

TEST(ObstacleMathTest, ProjectDistanceOnHorizontalPlane)
{
	// standard vehicle orientation inputs
	Quatf vehicle_pitch_up_45(Eulerf(0.0f, M_PI_4_F, 0.0f));
	Quatf vehicle_roll_right_45(Eulerf(M_PI_4_F, 0.0f, 0.0f));

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	float distance = 1.0f;
	float sensor_orientation = 0; // radians (forward facing)

	// WHEN: we project the distance onto the horizontal plane
	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_pitch_up_45);

	// THEN: the distance should be scaled correctly
	float expected_scale    = sqrtf(2) / 2;
	float expected_distance = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	distance = 1.0f;

	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_roll_right_45);

	// THEN: the distance should be scaled correctly
	expected_scale     = 1.f;
	expected_distance  = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	distance = 1.0f;
	sensor_orientation = M_PI_2_F; // radians (right facing)

	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_roll_right_45);

	// THEN: the distance should be scaled correctly
	expected_scale     = sqrtf(2) / 2;
	expected_distance  = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	distance = 1.0f;

	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_pitch_up_45);

	// THEN: the distance should be scaled correctly
	expected_scale     = 1.f;
	expected_distance  = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

}
