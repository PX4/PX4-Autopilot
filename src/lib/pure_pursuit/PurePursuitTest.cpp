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

/******************************************************************
 * Test code for the Pure Pursuit algorithm
 * Run this test only using "make tests TESTFILTER=PurePursuit"
 *
 * Graphic interpretation:
 * 	Legend:
 * 		C: Current waypoint
 * 		P: Previous waypoint
 * 		V: Vehicle
 * 		|: Line segment
 * 	Orientation:
 * 							C
 * 		  				       /
 * 						    __/__
 * 						  /  /    \
 * 						 /  /      \
 * 						|  /  V     |
 * 						 \/        /
 * 						 /\ _____ /
 * 	         	    N (0 rad)		/
 * 			        ^	       P
 * 				|
 * 				| D
 *  	   (-1.5708 rad) <----- ⨂ -----> E (1.5708 rad)
 * 				|
 * 				|
 * 				⌄
 * 			(+- 3.14159 rad)
 *
 * NOTE:
 * 	The tuning parameters for the pure pursuit algorithm are set to the following for all tests:
 * 	   PP_LOOKAHD_GAIN = 1.f
 * 	   PP_LOOKAHD_MAX = 10.f
 * 	   PP_LOOKAHD_MIN = 1.f
 * 	This way passing the vehicle_speed in calcDesiredHeading function is equivalent to passing
 * 	the lookahead distance.
 *
******************************************************************/

#include <gtest/gtest.h>
#include <lib/pure_pursuit/PurePursuit.hpp>
#include <uORB/topics/pure_pursuit_status.h>

using namespace matrix;

TEST(PurePursuitTest, InvalidSpeed)
{
	//      V   C
	//         /
	//  	  /
	//	 /
	//	P
	pure_pursuit_status_s pure_pursuit{};
	const Vector2f curr_wp_ned(10.f, 10.f);
	const Vector2f prev_wp_ned(0.f, 0.f);
	const Vector2f curr_pos_ned(10.f, 0.f);
	// NaN speed
	const float target_bearing1 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, curr_wp_ned, prev_wp_ned,
				      curr_pos_ned, NAN);
	EXPECT_FALSE(PX4_ISFINITE(target_bearing1));
}

TEST(PurePursuitTest, InvalidWaypoints)
{
	//	V   C
	//         /
	//  	  /
	//	 /
	//	P
	pure_pursuit_status_s pure_pursuit{};
	const Vector2f curr_wp_ned(10.f, 10.f);
	const Vector2f prev_wp_ned(0.f, 0.f);
	const Vector2f curr_pos_ned(10.f, 0.f);
	const float lookahead_distance{5.f};
	// Prev WP is NAN
	const float target_bearing1 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, curr_wp_ned, Vector2f(NAN,
				      NAN), curr_pos_ned,
				      lookahead_distance);
	// Curr WP is NAN
	const float target_bearing2 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(NAN, NAN),
				      prev_wp_ned, curr_pos_ned,
				      lookahead_distance);

	// Curr Pos is NAN
	const float target_bearing3 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, curr_wp_ned, prev_wp_ned,
				      Vector2f(NAN, NAN), lookahead_distance);
	EXPECT_FALSE(PX4_ISFINITE(target_bearing1));
	EXPECT_FALSE(PX4_ISFINITE(target_bearing2));
	EXPECT_FALSE(PX4_ISFINITE(target_bearing3));
}

TEST(PurePursuitTest, OutOfLookahead)
{
	pure_pursuit_status_s pure_pursuit{};
	const float lookahead_distance{5.f};
	//	V   C
	//         /
	//  	  /
	//	 /
	//	P
	const float target_bearing1 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(10.f, 10.f),
				      Vector2f(0.f, 0.f), Vector2f(10.f, 0.f), lookahead_distance);
	//	    V
	//
	//	P ----- C
	const float target_bearing2 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				      Vector2f(0.f, 0.f), Vector2f(10.f, 10.f), lookahead_distance);
	// V
	//
	//     P ------ C
	const float target_bearing3 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				      Vector2f(0.f, 10.f), Vector2f(10.f, 0.f), lookahead_distance);
	//		V
	//
	// P ------ C
	const float target_bearing4 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 10.f),
				      Vector2f(0.f, 0.f), Vector2f(10.f, 20.f), lookahead_distance);

	EXPECT_NEAR(target_bearing1, M_PI_2_F + M_PI_4_F, FLT_EPSILON); // Fallback: Bearing to closest point on path
	EXPECT_NEAR(target_bearing2, -M_PI_F, FLT_EPSILON); 		// Fallback: Bearing to closest point on path
	EXPECT_NEAR(target_bearing3, M_PI_F - atan2f(10, 10), FLT_EPSILON); // Fallback: Bearing to previous waypoint
	EXPECT_NEAR(target_bearing4, -M_PI_F + atan2f(10, 10), FLT_EPSILON); // Fallback: Bearing to current waypoint
}

TEST(PurePursuitTest, WaypointOverlap)
{
	pure_pursuit_status_s pure_pursuit{};
	const float lookahead_distance{5.f};
	//	    C/P
	//
	//
	//
	//	V
	const float target_bearing1 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(10.f, 10.f),
				      Vector2f(10.f, 10.f), Vector2f(0.f, 0.f), lookahead_distance);
	//	    V
	//
	//
	//
	//	C/P
	const float target_bearing2 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 0.f),
				      Vector2f(0.f, 0.f), Vector2f(10.f, 10.f), lookahead_distance);
	EXPECT_NEAR(target_bearing1, M_PI_4_F, FLT_EPSILON); 		   // Fallback: Bearing to closest point on path
	EXPECT_NEAR(target_bearing2, -(M_PI_4_F + M_PI_2_F), FLT_EPSILON); // Fallback: Bearing to closest point on path
}

TEST(PurePursuitTest, CurrAndPrevSameNorthCoordinate)
{
	pure_pursuit_status_s pure_pursuit{};
	const float lookahead_distance{5.f};
	//	P -- V -- C
	const float target_bearing1 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				      Vector2f(0.f, 0.f), Vector2f(0.f, 10.f), lookahead_distance);

	//	     V
	//	P ------ C
	const float target_bearing2 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				      Vector2f(0.f, 0.f), Vector2f(5.f / sqrtf(2.f), 10.f), lookahead_distance);
	//	     V
	//	C ------ P
	const float target_bearing3 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 0.f),
				      Vector2f(0.f, 20.f), Vector2f(5.f / sqrtf(2.f), 10.f), lookahead_distance);
	//	     V
	//
	//	P ------ C
	const float target_bearing4 = PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				      Vector2f(0.f, 0.f), Vector2f(10.f, 10.f), lookahead_distance);

	EXPECT_NEAR(target_bearing1, M_PI_2_F, FLT_EPSILON);
	EXPECT_NEAR(target_bearing2, M_PI_2_F + M_PI_4_F, FLT_EPSILON);
	EXPECT_NEAR(target_bearing3, -(M_PI_2_F + M_PI_4_F), FLT_EPSILON);
	EXPECT_NEAR(target_bearing4, -M_PI_F, FLT_EPSILON); // Fallback: Bearing to closest point on path
}

TEST(PurePursuitTest, CrosstrackError)
{
	pure_pursuit_status_s pure_pursuit{};
	const float lookahead_distance{5.f};
	//	    V
	//
	//	P ----- C
	PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				       Vector2f(0.f, 0.f), Vector2f(10.f, 10.f), lookahead_distance);
	const float crosstrack1 = pure_pursuit.crosstrack_error;

	//	    V
	//	P ----- C
	PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				       Vector2f(0.f, 0.f), Vector2f(5.f, 10.f), lookahead_distance);
	const float crosstrack2 = pure_pursuit.crosstrack_error;

	//	P ----- C
	//
	//	    V
	PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				       Vector2f(0.f, 0.f), Vector2f(-10.f, 10.f), lookahead_distance);
	const float crosstrack3 = pure_pursuit.crosstrack_error;

	//	P ----- C
	//	    V
	PurePursuit::calcTargetBearing(pure_pursuit, 1.f, 10.f, 1.f, Vector2f(0.f, 20.f),
				       Vector2f(0.f, 0.f), Vector2f(-5.f, 10.f), lookahead_distance);
	const float crosstrack4 = pure_pursuit.crosstrack_error;

	EXPECT_NEAR(crosstrack1, -10.f, FLT_EPSILON);
	EXPECT_NEAR(crosstrack2, -5.f, FLT_EPSILON);
	EXPECT_NEAR(crosstrack3, 10.f, FLT_EPSILON);
	EXPECT_NEAR(crosstrack4, 5.f, FLT_EPSILON);

}
