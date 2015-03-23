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
 * @file integrator.h
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

 #pragma once

class Integrator {
public:
	Integrator(uint64_t auto_reset_interval = 0);
	virtual ~Integrator();

	/**
	 * Put an item into the integral.
	 *
	 * @param val		Item to put
	 * @return		true if putting the item triggered an integral reset
	 *			and the integral should be published
	 */
	bool			put(uint64_t timestamp, float val);

	/**
	 * Get the current integral value
	 *
	 * @return		the integral since the last auto-reset
	 */
	float			get() { return _integral_auto; }

	/**
	 * Read from the integral
	 *
	 * @param auto_reset	Reset the integral to zero on read
	 * @return		the integral since the last read-reset
	 */
	float			read(bool auto_reset);

private:
	uint64_t _auto_reset_interval;		/**< the interval after which the content will be published and the integrator reset */
	uint64_t _last_integration;		/**< timestamp of the last integration step */
	uint64_t _last_auto;			/**< last auto-announcement of integral value */
	float _integral_auto;			/**< the integrated value which auto-resets after _auto_reset_interval */
	float _integral_last_read;		/**< the integrated value since the last read */
	& _auto_callback;			/**< the function callback for auto-reset */

	/* we don't want this class to be copied */
	Integrator(const Integrator&);
	Integrator operator=(const Integrator&);
};

Integrator(auto_callback = nullptr, uint64_t auto_reset_interval = 0) :
	_auto_reset_interval(4000 /* 250 Hz */),
	_last_integration(0),
	_integral_auto(0.0f),
	_integral_last_read(0.0f),
	_auto_callback(auto_callback)
{

}

~Integrator()
{

}

bool
Integrator::put(uint64_t timestamp, float val)
{
	if (_last_integration == 0) {
		/* this is the first item in the integrator */
		_last_integration = timestamp;
		_last_auto = timestamp;
		return false;
	}

	float dt = (_last_integration - timestamp) / 1000000.0f;

	float i = dt * val;

	_integral_auto += i;
	_integral_last_read += i;

	_last_integration = timestamp;

	if (_auto_callback &&
		((_last_integration - _last_auto) > _auto_reset_interval)) {
		/* call the callback */
		_auto_callback(timestamp, _integral_auto);
		_last_auto = timestamp;
		_integral_auto = 0.0f;
	}
}

// XXX this should interpolate
float
Integrator::read(bool auto_reset)
{
	float val = _integral_read;
	if (auto_reset) {
		_integral_read = 0.0f;
	}

	return val;
}
