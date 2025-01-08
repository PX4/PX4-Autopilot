/****************************************************************************
 *
 * Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "CourseToAirspeedRefMapper.hpp"
using matrix::Vector2f;

float
CourseToAirspeedRefMapper::mapCourseSetpointToHeadingSetpoint(const float bearing_setpoint, const Vector2f &wind_vel,
		float airspeed_true) const
{

	Vector2f bearing_vector = Vector2f{cosf(bearing_setpoint), sinf(bearing_setpoint)};
	const float wind_cross_bearing = wind_vel.cross(bearing_vector);
	const float wind_dot_bearing = wind_vel.dot(bearing_vector);

	const Vector2f air_vel_ref = refAirVelocity(wind_vel, bearing_vector, wind_cross_bearing,
				     wind_dot_bearing, wind_vel.norm(), airspeed_true);

	return atan2f(air_vel_ref(1), air_vel_ref(0));
}

matrix::Vector2f CourseToAirspeedRefMapper::refAirVelocity(const Vector2f &wind_vel, const Vector2f &bearing_vec,
		const float wind_cross_bearing, const float wind_dot_bearing,
		const float wind_speed, float airspeed_true) const
{
	Vector2f air_vel_ref;

	if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_true, wind_speed)) {
		const float airsp_dot_bearing = projectAirspOnBearing(airspeed_true, wind_cross_bearing);
		air_vel_ref = solveWindTriangle(wind_cross_bearing, airsp_dot_bearing, bearing_vec);

	} else {
		air_vel_ref = infeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_true);
	}

	return air_vel_ref;
}

float CourseToAirspeedRefMapper::projectAirspOnBearing(const float airspeed, const float wind_cross_bearing) const
{
	// NOTE: wind_cross_bearing must be less than airspeed to use this function
	// it is assumed that bearing feasibility is checked and found feasible (e.g. bearingIsFeasible() = true) prior to entering this method
	// otherwise the return will be erroneous
	return sqrtf(math::max(airspeed * airspeed - wind_cross_bearing * wind_cross_bearing, 0.0f));
}

int CourseToAirspeedRefMapper::bearingIsFeasible(const float wind_cross_bearing, const float wind_dot_bearing,
		const float airspeed, const float wind_speed) const
{
	return (fabsf(wind_cross_bearing) < airspeed) && ((wind_dot_bearing > 0.0f) || (wind_speed < airspeed));
}

matrix::Vector2f
CourseToAirspeedRefMapper::solveWindTriangle(const float wind_cross_bearing, const float airsp_dot_bearing,
		const Vector2f &bearing_vec) const
{
	// essentially a 2D rotation with the speeds (magnitudes) baked in
	return Vector2f{airsp_dot_bearing * bearing_vec(0) - wind_cross_bearing * bearing_vec(1),
			wind_cross_bearing * bearing_vec(0) + airsp_dot_bearing * bearing_vec(1)};
}

matrix::Vector2f CourseToAirspeedRefMapper::infeasibleAirVelRef(const Vector2f &wind_vel, const Vector2f &bearing_vec,
		const float wind_speed, const float airspeed) const
{
	// NOTE: wind speed must be greater than airspeed, and airspeed must be greater than zero to use this function
	// it is assumed that bearing feasibility is checked and found infeasible (e.g. bearingIsFeasible() = false) prior to entering this method
	// otherwise the normalization of the air velocity vector could have a division by zero
	Vector2f air_vel_ref = sqrtf(math::max(wind_speed * wind_speed - airspeed * airspeed, 0.0f)) * bearing_vec - wind_vel;
	return air_vel_ref.normalized() * airspeed;
}
