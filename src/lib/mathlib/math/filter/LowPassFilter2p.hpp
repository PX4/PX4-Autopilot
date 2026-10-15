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

/// @file	LowPassFilter2p.hpp
/// @brief	A class to implement a second order low pass filter
/// Author: Leonard Hall <LeonardTHall@gmail.com>
/// Adapted for PX4 by Andrew Tridgell

#pragma once

#include <mathlib/math/Functions.hpp>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class LowPassFilter2p
{
public:
	LowPassFilter2p() = default;

	LowPassFilter2p(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq)
	{
		if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2)
		    || !isFinite(sample_freq) || !isFinite(cutoff_freq)) {

			disable();
			return;
		}

		// reset delay elements on filter change
		_delay_element_1 = {};
		_delay_element_2 = {};

		_cutoff_freq = math::max(cutoff_freq, sample_freq * 0.001f);
		_sample_freq = sample_freq;

		const float fr = _sample_freq / _cutoff_freq;
		const float ohm = tanf(M_PI_F / fr);
		const float c = 1.f + 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm;

		_b0 = ohm * ohm / c;
		_b1 = 2.f * _b0;
		_b2 = _b0;

		_a1 = 2.f * (ohm * ohm - 1.f) / c;
		_a2 = (1.f - 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm) / c;

		if (!isFinite(_b0) || !isFinite(_b1) || !isFinite(_b2) || !isFinite(_a1) || !isFinite(_a2)) {
			disable();
		}
	}

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		// Direct Form II implementation
		T delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};

		T output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		return output;
	}

	// Filter array of samples in place using the Direct form II.
	inline void applyArray(T samples[], int num_samples)
	{
		Biquad filter{*this};

		for (int n = 0; n < num_samples; n++) {
			samples[n] = filter.apply(samples[n]);
		}

		filter.store(*this);
	}

	// Filter the arrays of samples of three filters in place, with the same results as applyArray() on each.
	// Interleaving the three filters lets their per-sample computations, each serial, overlap.
	static void applyArrays(LowPassFilter2p &filter0, T samples0[], LowPassFilter2p &filter1, T samples1[],
				LowPassFilter2p &filter2, T samples2[], int num_samples)
	{
		Biquad f0{filter0};
		Biquad f1{filter1};
		Biquad f2{filter2};

		for (int n = 0; n < num_samples; n++) {
			samples0[n] = f0.apply(samples0[n]);
			samples1[n] = f1.apply(samples1[n]);
			samples2[n] = f2.apply(samples2[n]);
		}

		f0.store(filter0);
		f1.store(filter1);
		f2.store(filter2);
	}

	// Return the cutoff frequency
	float get_cutoff_freq() const { return _cutoff_freq; }

	// Return the sample frequency
	float get_sample_freq() const { return _sample_freq; }

	float getMagnitudeResponse(float frequency) const;

	// Reset the filter state to this value
	T reset(const T &sample)
	{
		const T input = isFinite(sample) ? sample : T{};

		if (fabsf(1 + _a1 + _a2) > FLT_EPSILON) {
			_delay_element_1 = _delay_element_2 = input / (1 + _a1 + _a2);

			if (!isFinite(_delay_element_1) || !isFinite(_delay_element_2)) {
				_delay_element_1 = _delay_element_2 = input;
			}

		} else {
			_delay_element_1 = _delay_element_2 = input;
		}

		return apply(input);
	}

	void disable()
	{
		// no filtering
		_sample_freq = 0.f;
		_cutoff_freq = 0.f;

		_delay_element_1 = {};
		_delay_element_2 = {};

		_b0 = 1.f;
		_b1 = 0.f;
		_b2 = 0.f;

		_a1 = 0.f;
		_a2 = 0.f;
	}

protected:

	// The coefficients and the delay elements in local variables, to filter arrays: as members, they would be
	// reloaded and stored for every sample, since the samples may alias them.
	struct Biquad {
		explicit Biquad(const LowPassFilter2p &f) :
			a1(f._a1), a2(f._a2), b0(f._b0), b1(f._b1), b2(f._b2),
			delay_element_1(f._delay_element_1), delay_element_2(f._delay_element_2) {}

		// same computation as apply()
		T apply(const T &sample)
		{
			// Direct Form II implementation
			T delay_element_0{sample - delay_element_1 *a1 - delay_element_2 * a2};

			T output{delay_element_0 *b0 + delay_element_1 *b1 + delay_element_2 * b2};

			delay_element_2 = delay_element_1;
			delay_element_1 = delay_element_0;

			return output;
		}

		void store(LowPassFilter2p &f) const
		{
			f._delay_element_1 = delay_element_1;
			f._delay_element_2 = delay_element_2;
		}

		const float a1, a2, b0, b1, b2;
		T delay_element_1, delay_element_2;
	};

	T _delay_element_1{}; // buffered sample -1
	T _delay_element_2{}; // buffered sample -2

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};

	float _cutoff_freq{0.f};
	float _sample_freq{0.f};
};

} // namespace math
