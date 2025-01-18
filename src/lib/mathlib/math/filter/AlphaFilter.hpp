/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file AlphaFilter.hpp
 *
 * @brief First order "alpha" IIR digital filter also known as leaky integrator or forgetting average.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <float.h>
#include <mathlib/math/Functions.hpp>

using namespace math;

template <typename T>
class AlphaFilter
{
public:
	AlphaFilter() = default;
	explicit AlphaFilter(float sample_interval, float time_constant) { setParameters(sample_interval, time_constant); }
	explicit AlphaFilter(float time_constant) : _time_constant(time_constant) {};

	~AlphaFilter() = default;

	/**
	 * Set filter parameters for time abstraction
	 *
	 * Both parameters have to be provided in the same units.
	 *
	 * @param sample_interval interval between two samples in seconds
	 * @param time_constant filter time constant determining convergence in seconds
	 */
	void setParameters(float sample_interval, float time_constant)
	{
		const float denominator = time_constant + sample_interval;

		if (denominator > FLT_EPSILON) {
			setAlpha(sample_interval / denominator);
		}

		_time_constant = time_constant;
	}

	bool setCutoffFreq(float sample_freq, float cutoff_freq)
	{
		if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.f)
		    || !isFinite(sample_freq) || !isFinite(cutoff_freq)) {

			// Invalid parameters
			return false;
		}

		setParameters(1.f / sample_freq, 1.f / (M_TWOPI_F * cutoff_freq));
		return true;
	}

	void setCutoffFreq(float cutoff_freq)
	{
		if (cutoff_freq > FLT_EPSILON) {
			_time_constant = 1.f / (M_TWOPI_F * cutoff_freq);

		} else {
			_time_constant = 0.f;
		}
	}

	/**
	 * Set filter parameter alpha directly without time abstraction
	 *
	 * @param alpha [0,1] filter weight for the previous state. High value - long time constant.
	 */
	void setAlpha(float alpha) { _alpha = alpha; }

	/**
	 * Set filter state to an initial value
	 *
	 * @param sample new initial value
	 */
	void reset(const T &sample) { _filter_state = sample; }

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	const T &update(const T &sample)
	{
		_filter_state = updateCalculation(sample);
		return _filter_state;
	}

	const T update(const T &sample, float dt)
	{
		setParameters(dt, _time_constant);
		return update(sample);
	}

	const T &getState() const { return _filter_state; }
	float getCutoffFreq() const { return 1.f / (M_TWOPI_F * _time_constant); }

protected:
	T updateCalculation(const T &sample);

	float _time_constant{0.f};
	float _alpha{0.f};
	T _filter_state{};
};

template <typename T>
T AlphaFilter<T>::updateCalculation(const T &sample) { return _filter_state + _alpha * (sample - _filter_state); }

/* Specialization for 3D rotations
 * The filter is computed on the 3-sphere of unit quaternions instead of the cartesian space
 * Additions and subtractions are done using the quaternion multiplication and
 * the error is scaled on the tangent space.
 */
template <> inline
matrix::Quatf AlphaFilter<matrix::Quatf>::updateCalculation(const matrix::Quatf &sample)
{
	matrix::Quatf q_error(_filter_state.inversed() * sample);
	q_error.canonicalize(); // prevent unwrapping
	return _filter_state * matrix::Quatf(matrix::AxisAnglef(_alpha * matrix::AxisAnglef(q_error)));
}
