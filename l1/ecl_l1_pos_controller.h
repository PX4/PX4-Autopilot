/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_l1_pos_control.h
 * Implementation of L1 position control.
 *
 *
 * Acknowledgements and References:
 *
 *    This implementation has been built for PX4 based on the original
 *    publication from [1] and does include a lot of the ideas (not code)
 *    from [2].
 *
 *
 *    [1] S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 *    [2] Paul Riseborough, Brandon Jones and Andrew Tridgell, L1 control for APM. Aug 2013.
 *     - Explicit control over frequency and damping
 *     - Explicit control over track capture angle
 *     - Ability to use loiter radius smaller than L1 length
 *     - Modified to use PD control for circle tracking to enable loiter radius less than L1 length
 *     - Modified to enable period and damping of guidance loop to be set explicitly
 *     - Modified to provide explicit control over capture angle
 *
 */

#ifndef ECL_L1_POS_CONTROLLER_H
#define ECL_L1_POS_CONTROLLER_H

#include <mathlib/mathlib.h>
#include <geo/geo.h>
#include <ecl/ecl.h>

/**
 * L1 Nonlinear Guidance Logic
 */
class __EXPORT ECL_L1_Pos_Controller
{
public:
	ECL_L1_Pos_Controller() :
		_L1_distance(20.0),
		_L1_period(25.0),
		_L1_damping(0.75),
		_L1_ratio(5.0),
		_K_L1(2.0),
		_heading_omega(1.0),
		_roll_lim_rad(math::radians(10.0))
	{
	}

	/**
	 * The current target bearing
	 *
	 * @return bearing angle (-pi..pi, in NED frame)
	 */
	float nav_bearing();


	/**
	 * Get lateral acceleration demand.
	 *
	 * @return Lateral acceleration in m/s^2
	 */
	float nav_lateral_acceleration_demand();


	/**
	 * Heading error.
	 *
	 * The heading error is either compared to the current track
	 * or to the tangent of the current loiter radius.
	 */
	float bearing_error();


	/**
	 * Bearing from aircraft to current target.
	 *
	 * @return bearing angle (-pi..pi, in NED frame)
	 */
	float target_bearing();


	/**
	 * Get roll angle setpoint for fixed wing.
	 *
	 * @return Roll angle (in NED frame)
	 */
	float nav_roll();


	/**
	 * Get the current crosstrack error.
	 *
	 * @return Crosstrack error in meters.
	 */
	float crosstrack_error();


	/**
	 * Returns true if the loiter waypoint has been reached
	 */
	bool reached_loiter_target();


	/**
	 * Returns true if following a circle (loiter)
	 */
	bool circle_mode() {
		return _circle_mode;
	}


	/**
	 * Get the switch distance
	 * 
	 * This is the distance at which the system will
	 * switch to the next waypoint. This depends on the
	 * period and damping
	 *
	 * @param waypoint_switch_radius The switching radius the waypoint has set.
	 */
	float switch_distance(float waypoint_switch_radius);


	/**
	 * Navigate between two waypoints
	 *
	 * Calling this function with two waypoints results in the
	 * control outputs to fly to the line segment defined by
	 * the points and once captured following the line segment.
	 * This follows the logic in [1].
	 *
	 * @return sets _lateral_accel setpoint
	 */
	void navigate_waypoints(const math::Vector<2> &vector_A, const math::Vector<2> &vector_B, const math::Vector<2> &vector_curr_position,
			   const math::Vector<2> &ground_speed);


	/**
	 * Navigate on an orbit around a loiter waypoint.
	 *
	 * This allow orbits smaller than the L1 length,
	 * this modification was introduced in [2].
	 *
	 * @return sets _lateral_accel setpoint
	 */
	void navigate_loiter(const math::Vector<2> &vector_A, const math::Vector<2> &vector_curr_position, float radius, int8_t loiter_direction,
			   const math::Vector<2> &ground_speed_vector);


	/**
	 * Navigate on a fixed bearing.
	 *
	 * This only holds a certain direction and does not perform cross
	 * track correction. Helpful for semi-autonomous modes. Introduced
	 * by [2].
	 *
	 * @return sets _lateral_accel setpoint
	 */
	void navigate_heading(float navigation_heading, float current_heading, const math::Vector<2> &ground_speed);


	/**
	 * Keep the wings level.
	 *
	 * This is typically needed for maximum-lift-demand situations,
	 * such as takeoff or near stall. Introduced in [2].
	 */
	void navigate_level_flight(float current_heading);


	/**
	 * Set the L1 period.
	 */
	void set_l1_period(float period) {
		_L1_period = period;
		/* calculate the ratio introduced in [2] */
		_L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;
		/* calculate normalized frequency for heading tracking */
		_heading_omega = sqrtf(2.0f) * M_PI_F / _L1_period;
	}


	/**
	 * Set the L1 damping factor.
	 *
	 * The original publication recommends a default of sqrt(2) / 2 = 0.707
	 */
	void set_l1_damping(float damping) {
		_L1_damping = damping;
		/* calculate the ratio introduced in [2] */
		_L1_ratio = 1.0f / M_PI_F * _L1_damping * _L1_period;
		/* calculate the L1 gain (following [2]) */
		_K_L1 = 4.0f * _L1_damping * _L1_damping;
	}


	/**
	 * Set the maximum roll angle output in radians
	 *
	 */
	void set_l1_roll_limit(float roll_lim_rad) {
		_roll_lim_rad = roll_lim_rad;
	}

private:

	float _lateral_accel;		///< Lateral acceleration setpoint in m/s^2
	float _L1_distance;		///< L1 lead distance, defined by period and damping
	bool _circle_mode;		///< flag for loiter mode
	float _nav_bearing;		///< bearing to L1 reference point
	float _bearing_error;		///< bearing error
	float _crosstrack_error;	///< crosstrack error in meters
	float _target_bearing;		///< the heading setpoint

	float _L1_period;		///< L1 tracking period in seconds
	float _L1_damping;		///< L1 damping ratio
	float _L1_ratio;		///< L1 ratio for navigation
	float _K_L1;			///< L1 control gain for _L1_damping
	float _heading_omega;		///< Normalized frequency

	float _roll_lim_rad;  ///<maximum roll angle

	/**
	 * Convert a 2D vector from WGS84 to planar coordinates.
	 *
	 * This converts from latitude and longitude to planar
	 * coordinates with (0,0) being at the position of ref and
	 * returns a vector in meters towards wp.
	 *
	 * @param ref The reference position in WGS84 coordinates
	 * @param wp The point to convert to into the local coordinates, in WGS84 coordinates
	 * @return The vector in meters pointing from the reference position to the coordinates
	 */
	math::Vector<2> get_local_planar_vector(const math::Vector<2> &origin, const math::Vector<2> &target) const;

};


#endif /* ECL_L1_POS_CONTROLLER_H */
