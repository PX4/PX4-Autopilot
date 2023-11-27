/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <float.h>
#include <matrix/matrix/helper_functions.hpp>
#include "VelocitySmoothing.hpp"

/**
 * @brief Wrapper class for smoothing heading via maximum angular acceleration limited trajectories.
 *
 * Make sure to properly initialize the smoother on the first pass with:
 *
 * reset(initial heading, initial heading rate);
 * setMaxHeadingRate(max heading rate);
 * setMaxHeadingAccel(max heading accel);
 *
 * At the desired time interval, call the update method to update the smoother:
 * update(heading setpoint, time elapsed)
 *
 * Use the getters to retrieve the current smoothed setpoints.
 */
class HeadingSmoothing
{
public:
	HeadingSmoothing();
	~HeadingSmoothing() = default;

	/** @param max_heading_rate [rad/s] */
	void setMaxHeadingRate(const float max_heading_rate) { _velocity_smoothing.setMaxAccel(max_heading_rate); }

	/** @param max_heading_accel [rad/s^2] */
	void setMaxHeadingAccel(const float max_heading_accel) { _velocity_smoothing.setMaxJerk(max_heading_accel); }

	/**
	 * @brief resets internal trajectory states, handles heading wrap
	 *
	 * @param heading [rad] [-pi,pi]
	 * @param heading_rate [rad/s]
	 */
	void reset(const float heading, const float heading_rate);

	/**
	 * @brief updates the heading setpoint, re-calculates trajectory, and takes an integration step
	 *
	 * @param heading_setpoint [rad]
	 * @param time_elapsed [s]
	 */
	void update(const float heading_setpoint, const float time_elapsed);

	/** @return [rad] [-pi,pi] smoothed heading setpoint to retreive after update */
	float getSmoothedHeading() const { return _velocity_smoothing.getCurrentVelocity(); }

	/** @return [rad/s] smoothed heading rate setpoint to retreive after update */
	float getSmoothedHeadingRate() const { return _velocity_smoothing.getCurrentAcceleration(); }

private:
	VelocitySmoothing _velocity_smoothing;
};
