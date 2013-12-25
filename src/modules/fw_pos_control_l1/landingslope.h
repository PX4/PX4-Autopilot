/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
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

/*
 * @file: landingslope.h
 *
 */

#ifndef LANDINGSLOPE_H_
#define LANDINGSLOPE_H_

#include <math.h>
#include <systemlib/err.h>

class Landingslope
{
private:
	//xxx: documentation of landing pending
	float _landing_slope_angle_rad;
	float _flare_relative_alt;
	float _motor_lim_horizontal_distance;
	float _H1_virt;
	float _H0;
	float _d1;
	float _flare_constant;
	float _flare_length;
	float _horizontal_slope_displacement;

	void calculateSlopeValues();

public:
	Landingslope() {}
	~Landingslope() {}

	float getLandingSlopeAbsoluteAltitude(float wp_landing_distance, float wp_landing_altitude);

	/**
	 *
	 * @return Absolute altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
	 */
	__EXPORT static float getLandingSlopeAbsoluteAltitude(float wp_landing_distance, float wp_landing_altitude, float horizontal_slope_displacement, float landing_slope_angle_rad)
	{
		return (wp_landing_distance - horizontal_slope_displacement) * tanf(landing_slope_angle_rad) + wp_landing_altitude; //flare_relative_alt is negative
	}

	/**
	 *
	 * @return distance to landing waypoint of point on landing slope at altitude=slope_altitude
	 */
	__EXPORT static float getLandingSlopeWPDistance(float slope_altitude, float wp_landing_altitude, float horizontal_slope_displacement, float landing_slope_angle_rad)
	{
		return (slope_altitude - wp_landing_altitude)/tanf(landing_slope_angle_rad) + horizontal_slope_displacement;

	}


	float getFlareCurveAltitude(float wp_distance, float wp_altitude);

	void update(float landing_slope_angle_rad,
			float flare_relative_alt,
			float motor_lim_horizontal_distance,
			float H1_virt);


	inline float landing_slope_angle_rad() {return _landing_slope_angle_rad;}
	inline float flare_relative_alt() {return _flare_relative_alt;}
	inline float motor_lim_horizontal_distance() {return _motor_lim_horizontal_distance;}
	inline float H1_virt() {return _H1_virt;}
	inline float H0() {return _H0;}
	inline float d1() {return _d1;}
	inline float flare_constant() {return _flare_constant;}
	inline float flare_length() {return _flare_length;}
	inline float horizontal_slope_displacement() {return _horizontal_slope_displacement;}

};


#endif /* LANDINGSLOPE_H_ */
