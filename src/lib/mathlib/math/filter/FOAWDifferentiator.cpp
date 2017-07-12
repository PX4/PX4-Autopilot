// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file    FOAWDifferentiator.cpp
 * @brief   A class to implement a first order adaptive windowing differentiator
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * From:  Discrete-Time Adaptive Windowing for Velocity Estimation
 * Farrokh Janabi-Sharifi, Vincent Hayward, and Chung-Shin J. Chen
 */

#include <px4_defines.h>
#include "FOAWDifferentiator.hpp"
#include <cmath>

namespace math
{

FOAWDifferentiator::FOAWDifferentiator(float dt, float noise_level)
{
	set_noise_level(noise_level);
	set_sample_time(dt);
	reset();
}

FOAWDifferentiator::~FOAWDifferentiator()
{

}

void FOAWDifferentiator::set_noise_level(float noise_level)
{
	_delta = noise_level;
}

void FOAWDifferentiator::set_sample_time(float dt)
{
	_dt = dt;
}

uint8_t FOAWDifferentiator::get_last_window_size()
{
	return _last_window_size;
}

float FOAWDifferentiator::get_last_derivative()
{
	return _fit_val.a;
}

void FOAWDifferentiator::reset()
{
	memset(&_buffer, 0, sizeof(_buffer));
	_nb_samples = 0;
	_last_window_size = 0;
}

void FOAWDifferentiator::add_sample(float sample)
{
	if (_nb_samples <= _max_window_size) {
		_nb_samples++;

	} else {
		shift_buffer();
		_nb_samples = _max_window_size + 1;
	}

	_buffer[_nb_samples - 1] = sample;
}

void FOAWDifferentiator::shift_buffer()
{
	for (int i = 0; i < (_nb_samples - 1); i++) {
		_buffer[i] = _buffer[i + 1];
	}
}

void FOAWDifferentiator::end_fit_FOAW(uint8_t window_size)
{
	float d_amplitude;
	float d_time;
	uint8_t last_sample_pos;

	last_sample_pos = _nb_samples - 1;

	d_amplitude = _buffer[last_sample_pos] - _buffer[last_sample_pos - window_size];
	d_time  = window_size * _dt;
	_fit_val.a = d_amplitude / d_time;
	_fit_val.b = _buffer[last_sample_pos];
}

void FOAWDifferentiator::best_fit_FOAW(uint8_t window_size)
{
	float sum1;
	float sum2;
	float den;
	float y_mean;
	uint8_t last_sample_pos;
	uint8_t i;

	sum1 = 0.0f;
	sum2 = 0.0f;
	den = 0.0f;
	y_mean = 0.0f;
	_fit_val.a = 0.0f;
	_fit_val.b = 0.0f;

	last_sample_pos = _nb_samples - 1;

	// First order least squares fit of all the points inside the window
	for (i = 0; i <= window_size; i++) {
		sum1 += _buffer[last_sample_pos - i];
		sum2 += _buffer[last_sample_pos - i] * i;
	}

	y_mean = sum1 / (window_size + 1);
	sum1 *= window_size;
	sum2 *= 2;

	den = _dt * window_size * (window_size + 1) * (window_size + 2) / 6.0f;

	// Prevents division by zero
	if (den < 0.0001f && den > -0.0001f) {
		_fit_val.a = 0.0f;

	} else {
		_fit_val.a = (sum1 - sum2) / den;
	}

	_fit_val.b = y_mean + _fit_val.a * window_size * _dt / 2.0f;
}

// TODO; Add a way to be able to select the method you prefer (End-fit is faster but less accurate)
// Performs the Best-fit-R algorithm
float FOAWDifferentiator::fit()
{
	uint8_t window_size;
	uint8_t j;
	uint8_t last_sample_pos;
	uint8_t pass;
	float pos;
	float result;
	float slope;

	last_sample_pos = _nb_samples - 1;
	pass = 0;
	pos = 0.0f;
	result = 0.0f;
	slope = 0.0f;
	window_size = 1;

	//end_fit_FOAW(window_size);
	best_fit_FOAW(window_size);
	slope = _fit_val.a;
	result = slope;
	_last_window_size = window_size;

	if (last_sample_pos == 0) {
		return 0.0f;
	}

	for (window_size = 2; window_size <= (_nb_samples - 1); window_size++) {
		//end_fit_FOAW(window_size);
		best_fit_FOAW(window_size);
		slope = _fit_val.a;

		// Check if all the values are around the fit +/- delta
		for (j = 1; j < window_size; j++) {
			// Compute a point on the slope
			pos = _fit_val.b - slope * j * _dt;

			// Compute min and max bounds
			float max_bound = pos + _delta;
			float min_bound = pos - _delta;
			// Select sample to check
			float sample_to_check = _buffer[last_sample_pos - j];

			// Pass the test if the sample is inside the boundaries
			if (sample_to_check <= max_bound  && sample_to_check >= min_bound) {
				pass++;

			} else {
				break;
			}
		}

		// If all the values inside the windw are inside the boundaries, accept the new slope and continue with a bigger window if possible
		if (pass == (window_size - 1)) {
			result = slope;
			pass = 0;
			_last_window_size = window_size;
		}

		// Otherwise (at least one sample is outside) we keep the previous slope
		else {
			break;
		}

	}

	return result;
}


float FOAWDifferentiator::apply(float sample)
{
	float derivative;

	add_sample(sample);
	derivative = fit();

	return derivative;
}

} // namespace math

