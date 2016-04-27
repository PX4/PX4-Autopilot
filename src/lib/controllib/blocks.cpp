/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file blocks.cpp
 *
 * Controller library code
 */

#include <math.h>
#include <stdio.h>
#include <float.h>

#include "blocks.hpp"

#define ASSERT_CL(T) if (!(T)) { printf("FAIL\n"); return -1; }

namespace control
{

float BlockLimit::update(float input)
{
	if (input > getMax()) {
		input = _max.get();

	} else if (input < getMin()) {
		input = getMin();
	}

	return input;
}

float BlockLimitSym::update(float input)
{
	if (input > getMax()) {
		input = _max.get();

	} else if (input < -getMax()) {
		input = -getMax();
	}

	return input;
}

float BlockLowPass::update(float input)
{
	if (!PX4_ISFINITE(getState())) {
		setState(input);
	}

	float b = 2 * float(M_PI) * getFCut() * getDt();
	float a = b / (1 + b);
	setState(a * input + (1 - a)*getState());
	return getState();
}

float BlockHighPass::update(float input)
{
	float b = 2 * float(M_PI) * getFCut() * getDt();
	float a = 1 / (1 + b);
	setY(a * (getY() + input - getU()));
	setU(input);
	return getY();
}

float BlockLowPass2::update(float input)
{
	if (!PX4_ISFINITE(getState())) {
		setState(input);
	}

	if (fabsf(_lp.get_cutoff_freq() - getFCutParam()) > FLT_EPSILON) {
		_lp.set_cutoff_frequency(_fs, getFCutParam());
	}

	_state = _lp.apply(input);
	return _state;
}

float BlockIntegral::update(float input)
{
	// trapezoidal integration
	setY(_limit.update(getY() + input * getDt()));
	return getY();
}

float BlockIntegralTrap::update(float input)
{
	// trapezoidal integration
	setY(_limit.update(getY() +
			   (getU() + input) / 2.0f * getDt()));
	setU(input);
	return getY();
}

float BlockDerivative::update(float input)
{
	float output;

	if (_initialized) {
		output = _lowPass.update((input - getU()) / getDt());

	} else {
		// if this is the first call to update
		// we have no valid derivative
		// and so we use the assumption the
		// input value is not changing much,
		// which is the best we can do here.
		_lowPass.update(0.0f);
		output = 0.0f;
		_initialized = true;
	}

	setU(input);
	return output;
}

} // namespace control
