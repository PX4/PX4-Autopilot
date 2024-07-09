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

/**
 * @file FilteredDerivative.hpp
 *
 * @brief Derivative function passed through a first order "alpha" IIR digital filter
 *
 * @author Silvan Fuhrer <silvan@auterion.com>
 */

#pragma once

// #include <float.h>
// #include <mathlib/math/Functions.hpp>
#include <mathlib/math/filter/AlphaFilter.hpp>

using namespace math;

template <typename T>
class FilteredDerivative
{
public:
	FilteredDerivative() = default;
	~FilteredDerivative() = default;

	/**
	 * Set filter parameters for time abstraction
	 *
	 * Both parameters have to be provided in the same units.
	 *
	 * @param sample_interval interval between two samples
	 * @param time_constant filter time constant determining convergence
	 */
	void setParameters(float sample_interval, float time_constant)
	{
		_alpha_filter.setParameters(sample_interval, time_constant);
		_sample_interval = sample_interval;
	}

	/**
	 * Set filter state to an initial value
	 *
	 * @param sample new initial value
	 */
	void reset(const T &sample)
	{
		_alpha_filter.reset(sample);
		_initialized = false;
	}

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	const T &update(const T &sample)
	{
		if (_initialized) {
			if (_sample_interval > FLT_EPSILON) {
				_alpha_filter.update((sample - _previous_sample) / _sample_interval);

			} else {
				_initialized = false;
			}

		} else {
			// don't update in the first iteration
			_initialized = true;
		}

		_previous_sample = sample;
		return _alpha_filter.getState();
	}

	const T &getState() const { return _alpha_filter.getState(); }


private:
	AlphaFilter<T> _alpha_filter;
	float _sample_interval{0.f};
	T _previous_sample{0.f};
	bool _initialized{false};
};
