/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file system_identification.cpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include "system_identification.hpp"

void SystemIdentification::reset(const matrix::Vector<float, 5> &id_state_init)
{
	_rls.reset(id_state_init);
	_u_lpf.reset(0.f);
	_u_lpf.reset(0.f);
	_u_hpf = 0.f;
	_y_hpf = 0.f;
	_u_prev = 0.f;
	_y_prev = 0.f;
	_fitness_lpf.reset(10.f);
	_are_filters_initialized = false;
}

void SystemIdentification::update(float u, float y)
{
	updateFilters(u, y);
	update();
}

void SystemIdentification::update()
{
	_rls.update(_u_hpf, _y_hpf);
	updateFitness();
}

void SystemIdentification::updateFilters(float u, float y)
{
	if (!_are_filters_initialized) {
		_u_lpf.reset(u);
		_y_lpf.reset(y);
		_u_hpf = 0.f;
		_y_hpf = 0.f;
		_u_prev = u;
		_y_prev = y;
		_are_filters_initialized = true;
		return;
	}

	const float u_lpf = _u_lpf.apply(u);
	const float y_lpf = _y_lpf.apply(y);
	_u_hpf = _alpha_hpf * _u_hpf + _alpha_hpf * (u_lpf - _u_prev);
	_y_hpf = _alpha_hpf * _y_hpf + _alpha_hpf * (y_lpf - _y_prev);

	_u_prev = u_lpf;
	_y_prev = y_lpf;
}

void SystemIdentification::updateFitness()
{
	const matrix::Vector<float, 5> &diff = _rls.getDiffEstimate();
	float sum = 0.f;

	for (size_t i = 0; i < 5; i++) {
		sum += diff(i);
	}

	if (_dt > FLT_EPSILON) {
		_fitness_lpf.update(sum / _dt);
	}
}
