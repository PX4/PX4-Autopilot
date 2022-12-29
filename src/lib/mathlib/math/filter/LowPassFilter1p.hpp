/****************************************************************************
 *
 *   Copyright (C) 2012-2021 PX4 Development Team. All rights reserved.
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

/// @file	LowPassFilter1p.hpp
/// @brief	A class to implement a first order low pass filter
/// Based on simple 1st order RC model - https://en.wikipedia.org/wiki/Low-pass_filter
/// Note - no sample frequency is specified - instead a dt must be passed with each call
///      - no checks are made regarding nyquist frequency

#pragma once

#include <mathlib/math/Functions.hpp>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class LowPassFilter1p
{
public:
	LowPassFilter1p() = default;

	LowPassFilter1p(float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(cutoff_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float cutoff_freq, bool reset_states = true)
	{
		_cutoff_freq = cutoff_freq;
		update_alpha(reset_states);
	}

	/**
	 * Add a new raw value to the filter with a time delta
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample, float dt)
	{
		float alpha = (_k > 0) ? _k * dt / (1 + _k * dt) : 1.f;
		_delay_element_1 += alpha * (sample - _delay_element_1);
		return _delay_element_1;
	}

	/**
	 * Get the latest value
	 *
	 * @return retrieve the latest value
	 */
	inline T get()
	{
		return _delay_element_1;
	}

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Reset the filter state to this value
	T reset(const T &sample)
	{
		_delay_element_1 = sample;
		return _delay_element_1;
	}

	void disable(bool reset_states = true)
	{
		// no filtering
		_k = 0.f;

		// optionally reset delay elements on disable
		if (reset_states) {
			_delay_element_1 = {};
		}
	}

	bool disabled()
	{
		return _k < 0.01f;
	}

protected:
	void update_alpha(bool reset_states)
	{
		if ((_cutoff_freq <= 0.f) || !isFinite(_cutoff_freq)) {
			disable(reset_states);
			return;
		}

		_k = 2 * M_PI_F * _cutoff_freq;

		if (!isFinite(_k) || (_k < 0.f)) {
			disable(reset_states);
			return;
		}

		// optionally reset delay elements on filter change
		if (reset_states) {
			_delay_element_1 = {};
		}
	}

protected:
	T _delay_element_1{}; 		// buffered output -1
	float _k{0.f}; 			// 2 * pi * cutoff frequency
	float _cutoff_freq{0.f};	// cutoff frequency - Hz
};

} // namespace math
