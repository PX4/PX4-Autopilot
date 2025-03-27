
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

/******************************************************************
 * Test code for the NPFG algorithm
 * Run this test only using "make tests TESTFILTER=Npfg"
 *
 *
 * NOTE:
 *
 *
******************************************************************/

#include <gtest/gtest.h>
#include <lib/npfg/CourseToAirspeedRefMapper.hpp>

using namespace matrix;

TEST(NpfgTest, Test)
{
	//      V   C
	//         /
	//  	  /
	//	 /
	//	P
	const Vector2f curr_wp_ned(10.f, 10.f);
	float target_bearing1 = NAN;
	// NaN speed
	EXPECT_FALSE(PX4_ISFINITE(target_bearing1));
}

TEST(NpfgTest, NoWind)
{
	CourseToAirspeedRefMapper _course_to_airspeed;

	// GIVEN
	const Vector2f wind_vel(0.f, 0.f);
	float bearing = 0.f;
	float airspeed_max = 20.f;
	float min_ground_speed = 5.0f;
	float airspeed_setpoint = 15.f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	float heading_setpoint = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel, airspeed_setpoint);
	float min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
					 airspeed_max, min_ground_speed);

	// THEN: expect heading due North with a min airspeed equal to min_ground_speed
	EXPECT_NEAR(heading_setpoint, 0.f, FLT_EPSILON);
	EXPECT_NEAR(min_airspeed_for_bearing, min_ground_speed, FLT_EPSILON);

	// GIVEN: bearing due South
	bearing = M_PI_F;
	airspeed_max = 20.f;
	min_ground_speed = 5.0f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	heading_setpoint = matrix::wrap_pi(_course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel,
					   airspeed_setpoint));
	min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
				   airspeed_max, min_ground_speed);

	// THEN: expect heading due South with a min airspeed equal to min_ground_speed
	EXPECT_NEAR(heading_setpoint, -M_PI_F, 2 * FLT_EPSILON); // Why is the 2*FLT_EPS required here to make it pass?
	EXPECT_NEAR(min_airspeed_for_bearing, min_ground_speed, FLT_EPSILON);
}

TEST(NpfgTest, LightCrossWind)
{
	CourseToAirspeedRefMapper _course_to_airspeed;

	// GIVEN
	const Vector2f wind_vel(0.f, 6.f);
	float bearing = 0.f;
	float airspeed_max = 20.f;
	float min_ground_speed = 5.0f;
	float airspeed_setpoint = 15.f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	float heading_setpoint = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel, airspeed_setpoint);
	float min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
					 airspeed_max, min_ground_speed);

	// THEN: expect heading -0.4115168 with a min airspeed of to 7.8 (sqrt(25+36))
	EXPECT_NEAR(heading_setpoint, -0.4115168, 0.1f);
	EXPECT_NEAR(min_airspeed_for_bearing, 7.8f, 0.1f);

	// GIVEN: bearing due South
	bearing = M_PI_F;
	airspeed_max = 20.f;
	min_ground_speed = 5.0f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	heading_setpoint = matrix::wrap_pi(_course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel,
					   airspeed_setpoint));
	min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
				   airspeed_max, min_ground_speed);

	// THEN: expect heading of -2.73 and a min airspeed of 7.8 (sqrt(25+36))
	EXPECT_NEAR(heading_setpoint, -2.73f, 0.1f); // Why is the 2*FLT_EPS required here to make it pass?
	EXPECT_NEAR(min_airspeed_for_bearing, 7.8f, 0.1f);
}

TEST(NpfgTest, StrongHeadWing)
{
	CourseToAirspeedRefMapper _course_to_airspeed;

	// GIVEN
	const Vector2f wind_vel(-16.f, 0.f);
	float bearing = 0.f;
	float airspeed_max = 25.f;
	float min_ground_speed = 5.0f;
	float airspeed_setpoint = 15.f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	float heading_setpoint = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel, airspeed_setpoint);
	float min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
					 airspeed_max, min_ground_speed);

	// THEN: expect heading due North with a min airspeed equal to 16+min_ground_speed
	EXPECT_NEAR(heading_setpoint, 0.f, 0.1f);
	EXPECT_NEAR(min_airspeed_for_bearing, 16 + min_ground_speed, 0.1f);

	// GIVEN: bearing due South
	bearing = M_PI_F;
	airspeed_max = 25.f;
	min_ground_speed = 5.0f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	heading_setpoint = matrix::wrap_pi(_course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel,
					   airspeed_setpoint));
	min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
				   airspeed_max, min_ground_speed);

	// THEN: expect heading due South with a min airspeed at 0
	EXPECT_NEAR(heading_setpoint, -M_PI_F, 0.1f); // Why is the 2*FLT_EPS required here to make it pass?
	EXPECT_NEAR(min_airspeed_for_bearing, 0.f, 0.1f);
}

TEST(NpfgTest, ExceedingHeadWind)
{
	CourseToAirspeedRefMapper _course_to_airspeed;

	// GIVEN
	const Vector2f wind_vel(-25.f, 0.f);
	float bearing = 0.f;
	float airspeed_max = 25.f;
	float min_ground_speed = 5.0f;
	float airspeed_setpoint = 15.f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	float heading_setpoint = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel, airspeed_setpoint);
	float min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
					 airspeed_max, min_ground_speed);

	// THEN: expect heading sp due North with a min airspeed equal to airspeed_max
	EXPECT_NEAR(heading_setpoint, 0.f, 0.1f);
	EXPECT_NEAR(min_airspeed_for_bearing, airspeed_max, 0.1f);

	// GIVEN: bearing due South
	bearing = M_PI_F;
	airspeed_max = 25.f;
	min_ground_speed = 5.0f;

	// WHEN: we update bearing and airspeed magnitude augmentation
	heading_setpoint = matrix::wrap_pi(_course_to_airspeed.mapCourseSetpointToHeadingSetpoint(bearing, wind_vel,
					   airspeed_setpoint));
	min_airspeed_for_bearing = _course_to_airspeed.getMinAirspeedForCurrentBearing(bearing, wind_vel,
				   airspeed_max, min_ground_speed);

	// THEN: expect heading due South with a min airspeed equal at 0
	EXPECT_NEAR(heading_setpoint, -M_PI_F, 0.1f); // Why is the 2*FLT_EPS required here to make it pass?
	EXPECT_NEAR(min_airspeed_for_bearing, 0.f, 0.1f);
}
