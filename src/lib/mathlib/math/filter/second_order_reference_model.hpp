/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file second_order_reference_model.hpp
 *
 * @brief Implementation of a second order system with optional rate feed-forward.
 * Note that sample time abstraction is not done here. Users should run the filter
 * at a sufficiently fast rate to achieve the continuous time performance described
 * by the natural frequency and damping ratio parameters. Tuning of these parameters
 * will only maintain a given performance for the sampled time for which it was tuned.
 *
 * @author Thomas Stastny <thomas.stastny@auterion.com>
 */

#pragma once

#include <px4_platform_common/defines.h>

namespace math
{

template <typename T>
class SecondOrderReferenceModel
{
public:
	SecondOrderReferenceModel() = default;

	/**
	 * Construct with system parameters
	 *
	 * @param[in] natural_freq The desired natural frequency of the system [rad/s]
	 * @param[in] damping_ratio The desired damping ratio of the system
	 */
	SecondOrderReferenceModel(const float natural_freq, const float damping_ratio)
	{
		setParameters(natural_freq, damping_ratio);
	}

	/**
	 * Set the system parameters
	 *
	 * Calculates the damping coefficient, spring constant, and maximum allowed
	 * time step based on the natural frequency.
	 *
	 * @param[in] natural_freq The desired natural frequency of the system [rad/s]
	 * @param[in] damping_ratio The desired damping ratio of the system
	 * @return Whether or not the param set was successful
	 */
	bool setParameters(const float natural_freq, const float damping_ratio)
	{
		if (natural_freq < FLT_EPSILON || damping_ratio < FLT_EPSILON) {

			// Deadzone the resulting constants (will result in zero filter acceleration)
			spring_constant_ = 0.0f;
			damping_coefficient_ = 0.0f;

			// Zero natural frequency means our time step is irrelevant
			max_time_step_ = INFINITY;

			// Fail
			return false;
		}

		// Note these are "effective" coefficients, as mass coefficient is considered baked in
		spring_constant_ = natural_freq * natural_freq;
		damping_coefficient_ = 2.0f * damping_ratio * natural_freq;

		// Based on *conservative nyquist frequency via SAMPLE_RATE_MULTIPLIER
		max_time_step_ = 2.0f * M_PI_F / (natural_freq * SAMPLE_RATE_MULTIPLIER);

		return true;
	}

	/**
	 * @return System state [units]
	 */
	const T &getState() const { return filter_state_; }

	/**
	 * @return System rate [units/s]
	 */
	const T &getRate() const { return filter_rate_; }

	/**
	 * @return System acceleration [units/s^2]
	 */
	const T &getAccel() const { return filter_accel_; }

	/**
	 * Update the system states
	 *
	 * Units of rate_sample must correspond to the derivative of state_sample units.
	 * There is currently no handling of discrete time behavior w.r.t. continuous time
	 * system parameters. Sampling time should be sufficiently fast.
	 *
	 * @param[in] time_step Time since last sample [s]
	 * @param[in] state_sample New state sample [units]
	 * @param[in] rate_sample New rate sample, if provided, otherwise defaults to zero(s) [units/s]
	 */
	void update(const float time_step, const T &state_sample, const T &rate_sample = T())
	{
		if (time_step > max_time_step_) {
			// time step is too large, reset the filter
			reset(state_sample, rate_sample);
			return;
		}

		if (time_step < 0.0f) {
			// erroneous input, dont update
			return;
		}

		filter_state_ = integrate(filter_state_, filter_rate_, time_step);
		filter_rate_ = integrate(filter_rate_, filter_accel_, time_step);

		T state_error = state_sample - filter_state_;
		T rate_error = rate_sample - filter_rate_;
		filter_accel_ = state_error * spring_constant_ + rate_error * damping_coefficient_;
	}

	/**
	 * Reset the system states
	 *
	 * @param[in] state Initial state [units]
	 * @param[in] rate Initial rate, if provided, otherwise defaults to zero(s) [units/s]
	 */
	void reset(const T &state, const T &rate = T())
	{
		filter_state_ = state;
		filter_rate_ = rate;
		filter_accel_ = T();
	}

protected:

	// A conservative multiplier on sample frequency to bound the maximum time step
	static constexpr float SAMPLE_RATE_MULTIPLIER = 10.0f;

	// (effective, no mass) Spring constant for second order system [s^-2]
	float spring_constant_{0.0f};

	// (effective, no mass) Damping coefficient for second order system [s^-1]
	float damping_coefficient_{0.0f};

	T filter_state_{}; // [units]
	T filter_rate_{}; // [units/s]
	T filter_accel_{}; // [units/s^2]

	// Maximum time step [s]
	float max_time_step_{INFINITY};

	/**
	 * Take one integration step using Euler integration
	 *
	 * @param[in] last_state [units]
	 * @param[in] rate [units/s]
	 * @param[in] time_step Time since last sample [s]
	 * @return The next state [units]
	 */
	T integrate(const T &last_state, const T &rate, const float time_step) const
	{
		return last_state + rate * time_step;
	}
};

} // namespace math
