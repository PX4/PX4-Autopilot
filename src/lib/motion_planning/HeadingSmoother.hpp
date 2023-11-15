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

/**
 * @file HeadingSmoother.hpp
 *
 */

#pragma once

#include <motion_planning/VelocitySmoothing.hpp>
#include <matrix/matrix/helper_functions.hpp>
#include <float.h>

/**
 * @brief Wrapper class for smoothing heading via maximum angular acceleration limited trajectories.
 *
 * If instantiating the class before smoothing limits and initial states are known, make sure to properly initialize the
 * smoother on the first pass with all of the following, default values cause very slow smoothing:
 *
 * reset(initial heading, initial heading rate);
 * setMaxHeadingRate(max heading rate);
 * setMaxHeadingAccel(max heading accel);
 *
 * At the desired time interval, call the update method to update the smoother:
 *
 * update(heading setpoint, time elapsed)
 *
 * Use the getters to retrieve the current smoothed states.
 */
class HeadingSmoother
{
public:
	/**
	 * @brief Construct with default smoothing initializations
	 *
	 */
	HeadingSmoother()
	{
		reset(0.f, 0.f);
		setMaxHeadingRate(VelocitySmoothing::kMinAccel);
		setMaxHeadingAccel(VelocitySmoothing::kMinJerk);
		_heading_smoother.setMaxVel(6.f * M_PI_F); // arbitrary large angle the wrapped value should never reach
	}

	/**
	 * @brief construct with explicit smoothing initializations
	 *
	 * @param initial_heading [rad] [-pi, pi]
	 * @param initial_heading_rate [rad/s]
	 * @param max_heading_rate [rad/s]
	 * @param max_heading_accel [rad/s^2]
	 */
	HeadingSmoother(const float initial_heading, const float initial_heading_rate, const float max_heading_rate,
			const float max_heading_accel)
	{
		reset(initial_heading, initial_heading_rate);
		setMaxHeadingRate(max_heading_rate); // is sanitized to lowest value in velocity smoothing setter
		setMaxHeadingAccel(max_heading_accel); // is sanitized to lowest value in velocity smoothing setter
		_heading_smoother.setMaxVel(6.f * M_PI_F); // arbitrary large angle the wrapped value should never reach
	}

	~HeadingSmoother() = default;

	/**
	 * @return [rad] [-pi,pi] smoothed heading
	 */
	float getSmoothedHeading() const { return _heading_smoother.getCurrentVelocity(); }

	/**
	 * @return [rad/s] smoothed heading rate
	 */
	float getSmoothedHeadingRate() const { return _heading_smoother.getCurrentAcceleration(); }

	/**
	 * @param max_heading_rate [rad/s]
	 */
	void setMaxHeadingRate(const float max_heading_rate) { _heading_smoother.setMaxAccel(max_heading_rate); }

	/**
	 * @param max_heading_accel [rad/s^2]
	 */
	void setMaxHeadingAccel(const float max_heading_accel) { _heading_smoother.setMaxJerk(max_heading_accel); }

	/**
	 * @brief updates the heading setpoint, re-calculates trajectory, and takes an integration step
	 *
	 * @param heading_setpoint [rad]
	 * @param time_elapsed [s]
	 */
	void update(const float heading_setpoint, const float time_elapsed)
	{
		const float delta_heading_wrapped = matrix::wrap_pi(heading_setpoint - getSmoothedHeading());
		const float unwrapped_heading_setpoint = delta_heading_wrapped + getSmoothedHeading();

		_heading_smoother.updateDurations(unwrapped_heading_setpoint);
		_heading_smoother.updateTraj(time_elapsed);

		const float wrapped_current_heading = matrix::wrap_pi(getSmoothedHeading());
		_heading_smoother.setCurrentVelocity(wrapped_current_heading);
	}

	/**
	 * @brief resets internal trajectory states, handles heading wrap
	 *
	 * @param heading [rad] [-pi,pi]
	 * @param heading_rate [rad/s]
	 */
	void reset(const float heading, const float heading_rate)
	{
		const float wrapped_heading = matrix::wrap_pi(heading);
		_heading_smoother.setCurrentVelocity(wrapped_heading);
		_heading_smoother.setCurrentAcceleration(heading_rate);
	}

	// [rad/s] minimum value of the smoother's maximum heading rate
	static constexpr float kMinHeadingRate = VelocitySmoothing::kMinAccel;

	// [rad/s^2] minimum value of the smoother's maximum heading acceleration
	static constexpr float kMinHeadingAccel = VelocitySmoothing::kMinJerk;

private:

	VelocitySmoothing _heading_smoother;
};
