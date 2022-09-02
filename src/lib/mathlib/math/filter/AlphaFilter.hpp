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
	explicit AlphaFilter(float alpha) : _alpha(alpha) {}

	~AlphaFilter() = default;

	// 0.01 to 10,000 Hz
	static constexpr float MIN_FREQ_HZ = 0.01f;    // 0.01 Hz or 100s interval
	static constexpr float MAX_FREQ_HZ = 10'000.f; // 10,000 Hz or 0.0001s interval

	static constexpr float min_interval_s() { return 1.f / MAX_FREQ_HZ; }
	static constexpr float max_interval_s() { return 1.f / MIN_FREQ_HZ; }

	/**
	 * Set filter parameter alpha directly without time abstraction
	 *
	 * @param alpha [0,1] filter weight for the previous state. High value - long time constant.
	 */
	bool setAlpha(const float alpha)
	{
		if (alpha < 0.f || alpha > 1.f || !PX4_ISFINITE(alpha)) {
			// Invalid parameters, disable filter
			disable();
			return false;
		}

		_alpha = alpha;
		return true;
	}

	/**
	 * Set filter parameters for time abstraction
	 *
	 * Both parameters have to be provided in the same units.
	 *
	 * @param sample_interval interval between two samples
	 * @param time_constant filter time constant determining convergence
	 */
	bool setParameters(const float sample_interval, const float time_constant)
	{
		if ((sample_interval <= 0.f) || !PX4_ISFINITE(sample_interval)
		    || (time_constant <= 0.f) || !PX4_ISFINITE(time_constant)) {
			// invalid parameters, disable filter
			disable();
			return false;
		}

		_sample_interval = math::constrain(sample_interval, min_interval_s(), max_interval_s());
		_time_constant = time_constant;

		const float alpha = _sample_interval / (_sample_interval + _time_constant);

		return setAlpha(alpha);
	}

	bool setSampleAndCutoffFrequency(const float sample_freq, const float cutoff_freq)
	{
		if ((sample_freq <= 0.f) || !PX4_ISFINITE(sample_freq)
		    || (cutoff_freq <= 0.f) || !PX4_ISFINITE(cutoff_freq)) {
			// invalid parameters, disable filter
			disable();
			return false;
		}

		const float sample_freq_constrained = math::constrain(sample_freq, MIN_FREQ_HZ, MAX_FREQ_HZ);
		const float cutoff_freq_constrained = math::constrain(cutoff_freq, MIN_FREQ_HZ, sample_freq_constrained / 2.f);

		_sample_interval = 1.f / sample_freq_constrained;
		_time_constant = 1.f / (2.f * M_PI_F * cutoff_freq_constrained);

		const float alpha = _sample_interval / (_sample_interval + _time_constant);

		return setAlpha(alpha);
	}

	bool setTimeConstant(const float time_constant)
	{
		if ((time_constant <= 0.f) || !PX4_ISFINITE(time_constant)) {
			// Invalid parameters, disable filter
			disable();
			return false;
		}

		if ((fabsf(time_constant - _time_constant) / _time_constant) < 0.01f) {
			// no change, do nothing
			return true;
		}

		// time constant changed, update alpha
		_time_constant = time_constant;
		const float alpha = _sample_interval / (_sample_interval + _time_constant);

		return setAlpha(alpha);
	}

	bool setCutoffFrequency(const float cutoff_freq)
	{
		const float time_constant = 1.f / (2.f * M_PI_F * cutoff_freq);
		return setTimeConstant(time_constant);
	}

	/**
	 * Set filter state to an initial value
	 *
	 * @param sample new initial value
	 */
	const T &reset(const T &sample = {})
	{
		_filter_state = sample;
		return _filter_state;
	}

	void disable()
	{
		_alpha = 1.f;
		_sample_interval = 1.f;
		_time_constant = 0.f;
	}

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

	const T &update(const T &sample, const float dt)
	{
		if ((fabsf(dt - _sample_interval) / _sample_interval) > 0.01f) {
			// update parameters if changed by more than 1%
			if (!setParameters(dt, _time_constant)) {
				// invalid new parameters, reset filter
				return reset(sample);
			}
		}

		_filter_state = updateCalculation(sample);
		return _filter_state;
	}

	const T &getState() const { return _filter_state; }

	float getCutoffFreq() const { return 1.f / (2.f * M_PI_F * _time_constant); }

	const float &getSampleInterval() const { return _sample_interval; }
	const float &getTimeConstant() const { return _time_constant; }

private:
	T updateCalculation(const T &sample)
	{
		// don't allow bad updates to propagate
		const T ret = (1.f - _alpha) * _filter_state + _alpha * sample;

		if (isFinite(ret)) {
			return ret;
		}

		return {};
	}

	T _filter_state{};
	float _alpha{1.f};

	float _sample_interval{1.f};
	float _time_constant{0.f};
};
