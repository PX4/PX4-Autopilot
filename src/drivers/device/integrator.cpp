/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file integrator.cpp
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "integrator.h"

Integrator::Integrator(uint64_t auto_reset_interval, bool coning_compensation) :
	_auto_reset_interval(auto_reset_interval),
	_last_integration(0),
	_last_auto(0),
	_integral_auto(0.0f, 0.0f, 0.0f),
	_integral_read(0.0f, 0.0f, 0.0f),
	_last_val(0.0f, 0.0f, 0.0f),
	_last_delta(0.0f, 0.0f, 0.0f),
	_auto_callback(nullptr),
	_coning_comp_on(coning_compensation)
{

}

Integrator::~Integrator()
{

}

bool
Integrator::put(uint64_t timestamp, math::Vector<3> &val, math::Vector<3> &integral, uint64_t &integral_dt)
{
	bool auto_reset = false;

	if (_last_integration == 0) {
		/* this is the first item in the integrator */
		_last_integration = timestamp;
		_last_auto = timestamp;
		_last_val = val;
		return false;
	}

	// Integrate
	double dt = (double)(timestamp - _last_integration) / 1000000.0;
	math::Vector<3> i = (val + _last_val) * dt * 0.5f;

	// Apply coning compensation if required
	if (_coning_comp_on) {
		// Coning compensation derived by Paul Riseborough and Jonathan Challinger,
		// following:
		// Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
		// Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf

		i += ((_integral_auto + _last_delta * (1.0f / 6.0f)) % i) * 0.5f;
	}

	_integral_auto += i;
	_integral_read += i;

	_last_integration = timestamp;
	_last_val = val;
	_last_delta = i;

	if ((timestamp - _last_auto) > _auto_reset_interval) {
		if (_auto_callback) {
			/* call the callback */
			_auto_callback(timestamp, _integral_auto);
		}

		integral = _integral_auto;
		integral_dt = (timestamp - _last_auto);

		auto_reset = true;
		_last_auto = timestamp;
		_integral_auto(0) = 0.0f;
		_integral_auto(1) = 0.0f;
		_integral_auto(2) = 0.0f;
	}

	return auto_reset;
}

math::Vector<3>
Integrator::read(bool auto_reset)
{
	math::Vector<3> val = _integral_read;

	if (auto_reset) {
		_integral_read(0) = 0.0f;
		_integral_read(1) = 0.0f;
		_integral_read(2) = 0.0f;
	}

	return val;
}
