/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/math.hpp>
#include <px4_platform_common/module_params.h>

using namespace matrix;

/**
 * @file PurePursuit.hpp
 *
 * Implementation of pure pursuit guidance logic.
 *
 * Acknowledgements and References:
 *
 *    This implementation has been built for PX4 based on the idea from [1] (not including any code).
 *
 *    [1] Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm
 * 	  (Techreport CMU-RI-TR-92-01).
 *
 * Pure pursuit is a path following algorithm that uses the intersection between the path and
 * a circle (the radius of which is referred to as lookahead distance) around the vehicle as
 * the target point for the vehicle.
 * The lookahead distance is defined as v * k.
 * 	v: Vehicle ground speed [m/s]
 * 	k: Tuning parameter
 * The lookahead distance is further constrained between an upper and lower threshhold.
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
 * Input:  Current/prev waypoint and the vehicle position in NED frame as well as the vehicle speed.
 * Output: Calculates the intersection points as described above and returns the heading towards the point that is closer to the current waypoint.
 */
class PurePursuit : public ModuleParams
{
public:
	PurePursuit(ModuleParams *parent);
	~PurePursuit() = default;

	/**
	 * @brief Return heading towards the intersection point between a circle with a radius of
	 * vehicle_speed * PP_LOOKAHD_GAIN around the vehicle and an extended line segment from the previous to the current waypoint.
	 * Exceptions:
	 * 	Will return heading towards the current waypoint if it is closer to the vehicle than the lookahead or if the waypoints overlap.
	 * 	Will return heading towards the closest point on the path if there are no intersection points (crosstrack error bigger than lookahead).
	 * 	Will return NAN if input is invalid.
	 * @param curr_wp_ned North/East coordinates of current waypoint in NED frame [m].
	 * @param prev_wp_ned North/East coordinates of previous waypoint in NED frame [m].
	 * @param curr_pos_ned North/East coordinates of current position of the vehicle in NED frame [m].
	 * @param vehicle_speed Vehicle speed [m/s].
	 * @param PP_LOOKAHD_GAIN Tuning parameter [-]
	 * @param PP_LOOKAHD_MAX Maximum lookahead distance [m]
	 * @param PP_LOOKAHD_MIN Minimum lookahead distance [m]
	 */
	float calcDesiredHeading(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned, const Vector2f &curr_pos_ned,
				 float vehicle_speed);

	float getLookaheadDistance() {return _lookahead_distance;};
	float getCrosstrackError() {return _curr_pos_to_path.norm();};
	float getDistanceOnLineSegment() {return _distance_on_line_segment.norm();};

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

	struct {
		param_t lookahead_gain;
		param_t lookahead_max;
		param_t lookahead_min;
	} _param_handles{};

	struct {
		float lookahead_gain{1.f};
		float lookahead_max{10.f};
		float lookahead_min{1.f};
	} _params{};
private:
	float _lookahead_distance{0.f}; // Radius of the circle around the vehicle
	Vector2f _distance_on_line_segment{}; // Projection of prev_wp_to_curr_pos onto prev_wp_to_curr_wp
	Vector2f _curr_pos_to_path{}; // Shortest vector from the current position to the path
};
